
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "gt911.h"
// #include "bsp_i2c.h"

static const char *TAG = "GT911";
/* Reference: https://github.com/goodix/goodix_gt9xx_public
 * Datasheet: https://github.com/hadess/gt9xx/blob/master/specifications/GT911%20Datasheet.pdf
 */


/* Register defines */
#define GT_REG_CMD  0x8040

#define GT_REG_CFG  0x8047
#define GT_REG_DATA 0x8140


// Write only registers
#define GT911_REG_COMMAND        0x8040
#define GT911_REG_LED_CONTROL    0x8041
#define GT911_REG_PROXIMITY_EN   0x8042

// Read/write registers
// The version number of the configuration file
#define GT911_REG_CONFIG_DATA  0x8047
// X output maximum value (LSB 2 bytes)
#define GT911_REG_MAX_X        0x8048
// Y output maximum value (LSB 2 bytes)
#define GT911_REG_MAX_Y        0x804A
// Maximum number of output contacts: 1~5 (4 bit value 3:0, 7:4 is reserved)
#define GT911_REG_MAX_TOUCH    0x804C

// Module switch 1
// 7:6 Reserved, 5:4 Stretch rank, 3 X2Y, 2 SITO (Single sided ITO touch screen), 1:0 INT Trigger mode */
#define GT911_REG_MOD_SW1      0x804D
// Module switch 2
// 7:1 Reserved, 0 Touch key */
#define GT911_REG_MOD_SW2      0x804E

// Number of debuffs fingers press/release
#define GT911_REG_SHAKE_CNT    0x804F

// ReadOnly registers (device and coordinates info)
// Product ID (LSB 4 bytes, GT9110: 0x06 0x00 0x00 0x09)
#define GTP_REG_VERSION           0x8140
// Firmware version (LSB 2 bytes)
#define GT911_REG_FW_VER       0x8144

// Current output X resolution (LSB 2 bytes)
#define GT911_READ_X_RES       0x8146
// Current output Y resolution (LSB 2 bytes)
#define GT911_READ_Y_RES       0x8148
// Module vendor ID
#define GTP_REG_SENSOR_ID   0x814A

#define GTP_READ_COORD_ADDR  0x814E

/* Commands for REG_COMMAND */
//0: read coordinate state
#define GT911_CMD_READ         0x00
// 1: difference value original value
#define GT911_CMD_DIFFVAL      0x01
// 2: software reset
#define GT911_CMD_SOFTRESET    0x02
// 3: Baseline update
#define GT911_CMD_BASEUPDATE   0x03
// 4: Benchmark calibration
#define GT911_CMD_CALIBRATE    0x04
// 5: Off screen (send other invalid)
#define GT911_CMD_SCREEN_OFF   0x05

/* When data needs to be sent, the host sends command 0x21 to GT9x,
 * enabling GT911 to enter "Approach mode" and work as a transmitting terminal */
#define GT911_CMD_HOTKNOT_TX   0x21


typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// 0x28/0x29 (0x14 7bit)
// 0xBA/0xBB (0x5D 7bit)
#define I2C_ADDR1 0x5D
#define I2C_ADDR2 0x14

#define MASK_BIT_8      0x80
#define GTP_TOOL_PEN    1
#define GTP_TOOL_FINGER 2

#define MAX_KEY_NUMS 4
#define GTP_CONFIG_MIN_LENGTH   186
#define GTP_CONFIG_MAX_LENGTH 240
#define GTP_ADDR_LENGTH       2

/***************************PART1:ON/OFF define*******************************/
#define GTP_DEBUG_ON          1
#define GTP_DEBUG_ARRAY_ON    0
#define GTP_DEBUG_FUNC_ON     0

#define GTP_DEFAULT_MAX_X    720    /* default coordinate max values */
#define GTP_DEFAULT_MAX_Y    1080
#define GTP_DEFAULT_MAX_WIDTH    1024
#define GTP_DEFAULT_MAX_PRESSURE 1024
#define GTP_DEFAULT_INT_TRIGGER  1 /* 1 rising, 2 falling */
#define GTP_MAX_TOUCH_ID     16

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    int pin_num_int;
    touch_panel_dir_t direction;
    uint16_t width;
    uint16_t height;
} gt911_dev_t;

static gt911_dev_t g_dev;

static esp_err_t gt911_calibration_run(const scr_driver_t *screen, bool recalibrate);

touch_panel_driver_t gt911_default_driver = {
    .init = gt911_init,
    .deinit = gt911_deinit,
    .calibration_run = gt911_calibration_run,
    .set_direction = gt911_set_direction,
    .read_point_data = gt911_sample,
};

#define GOODIX_CONTACT_SIZE   8
#define GOODIX_MAX_CONTACTS   5
uint8_t points[GOODIX_MAX_CONTACTS*GOODIX_CONTACT_SIZE]; //points buffer
struct GTPoint {
  // 0x814F-0x8156, ... 0x8176 (5 points) 
  uint8_t trackId;
  uint16_t x;
  uint16_t y;
  uint16_t area;
  uint8_t reserved;
};

void i2c_scan()
{
    uint8_t foundDevices = 0;
  	for(int address = 1; address < 127; address++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
		i2c_master_stop(cmd);
		if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
      foundDevices++;
		}
		i2c_cmd_link_delete(cmd);
	  }
    if (foundDevices == 0)
    {
      printf("-> found NO devices");
    }
}


// 0x28/0x29 (0x14 7bit)
#define GOODIX_I2C_ADDR_28  0x14
// 0xBA/0xBB (0x5D 7bit)
#define GOODIX_I2C_ADDR_BA  0x5D
uint8_t intPin = 5;
uint8_t rstPin = 23;
uint8_t i2cAddr = 0x5d;

void  msSleep(uint16_t milliseconds) {
   vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}
void  usSleep(uint16_t microseconds) {
  vTaskDelay(microseconds);
}
void  pinOut(uint8_t pin) {
  gpio_pad_select_gpio((gpio_num_t)pin);
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
}

void  pinIn(uint8_t pin) {
  gpio_pad_select_gpio((gpio_num_t)pin);
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
}

void  pinSet(uint8_t pin, uint8_t level) {
  gpio_set_level((gpio_num_t)pin, level);
}

void  pinHold(uint8_t pin) {
  gpio_set_level((gpio_num_t)pin, 0);
}

bool  pinCheck(uint8_t pin, uint8_t level) {
  return gpio_get_level((gpio_num_t)pin) == level;
}

bool  reset() {
  msSleep(1);

  pinOut(intPin);
  pinOut(rstPin);

  pinHold(intPin);
  pinHold(rstPin);

  /* begin select I2C slave addr */

  /* T2: > 10ms */
  msSleep(11);

  /* HIGH: 0x28/0x29 (0x14 7bit), LOW: 0xBA/0xBB (0x5D 7bit) */
  pinSet(intPin, i2cAddr == GOODIX_I2C_ADDR_28);

  /* T3: > 100us */
  usSleep(110);
  pinIn(rstPin);
  if (pinCheck(rstPin, 1))
   return false;

  /* T4: > 5ms */
  msSleep(6);
  pinHold(intPin);
  /* end select I2C slave addr */

  /* T5: 50ms */
  msSleep(51);
  pinIn(intPin); 
  gpio_set_pull_mode((gpio_num_t)intPin, GPIO_FLOATING); // INT pin has no pullups so simple set to floating input

//   attachInterrupt((gpio_num_t)intPin, (gpio_isr_t) _goodix_irq_handler); 
  return true;
}

uint8_t  gdx_write(uint16_t reg, uint8_t buf) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( i2cAddr << 1 ) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
  i2c_master_write_byte(cmd, reg >> 8, I2C_MASTER_ACK);
  i2c_master_write_byte(cmd, reg & 0xff, I2C_MASTER_ACK);

  ESP_ERROR_CHECK(i2c_master_write(cmd, &buf, (size_t)1, I2C_MASTER_ACK));
  i2c_master_stop(cmd);
  esp_err_t esp_i2c_err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100/ portTICK_RATE_MS);
  
  ESP_LOGD(TAG,"i2c master read error: %d", esp_i2c_err);
  i2c_cmd_link_delete(cmd);

  return esp_i2c_err;
}

uint8_t gdx_read(uint16_t reg, uint8_t *buffer, size_t len) {

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ESP_LOGD(TAG,": set reg");
    i2c_master_write_byte(cmd, (uint8_t)(( i2cAddr << 1 ) | I2C_MASTER_WRITE), I2C_MASTER_ACK);
     i2c_master_write_byte(cmd, (uint8_t)(reg >> 8), I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, (uint8_t)(reg & 0xff), I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    esp_err_t esp_i2c_err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/ portTICK_RATE_MS);
    ESP_LOGD(TAG,"i2c master read error: %d", esp_i2c_err);
    i2c_cmd_link_delete(cmd);
    // make a break and continue with the read
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)(( i2cAddr << 1 ) | I2C_MASTER_READ), I2C_MASTER_ACK);
    ESP_LOGD(TAG,": 0x5d << 1 ) | I2C_MASTER_READ");
    if (len > 1){
      i2c_master_read(cmd, buffer, ((size_t)len - 1) , I2C_MASTER_ACK);
    }
    ESP_LOGD(TAG,": requested len %d", len);
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &buffer[len-1], I2C_MASTER_NACK));
    i2c_master_stop(cmd);
    esp_i2c_err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/ portTICK_RATE_MS);
    ESP_LOGD(TAG,"i2c master read error: %d", esp_i2c_err);
    i2c_cmd_link_delete(cmd);

  return esp_i2c_err;

}

uint8_t dumpRegID(){
  uint8_t buffer[11];
  uint8_t i2c_err = gdx_read(GTP_REG_VERSION, buffer, 11);
  if (i2c_err != ESP_OK){
    ESP_LOGD(TAG,"---error---");
  }
  else{  
    ESP_LOGD(TAG,"-no--error--");
    printf("Product-ID: %c%c%c%c\r\n",buffer[0],buffer[1],buffer[2],buffer[3]);
    printf("Firmware-Version: %x%x\r\n",buffer[5],buffer[4]);
    uint16_t res = buffer[6] | buffer[7] << 8;
    printf("X-Resolution: %d\r\n",res);
    res = buffer[8] | buffer[9] << 8;
    printf("Y-Resolution: %d\r\n",res);
    printf("Vendor-ID: %x\r\n",buffer[10]);
    }
  return i2c_err;  
}

#define EAGAIN 100 // Try again error
int16_t  readInput(uint8_t *data) {
  int touch_num;
  int error;

  uint8_t regState[1];

  error = gdx_read(GTP_READ_COORD_ADDR, regState, 1);

  if (error) {
    return -error;
  }

  if (!(regState[0] & 0x80))
    return -EAGAIN;

  touch_num = regState[0] & 0x0f;

  if (touch_num > 0) {
      error = gdx_read(GTP_READ_COORD_ADDR + 1, data, GTP_READ_COORD_ADDR * (touch_num));

    if (error)
      return -error;
  }

  return touch_num;
}

void handleTouch(int8_t contacts, struct GTPoint *points) {
  ESP_LOGD(TAG,"Contacts: %d", contacts);
  for (uint8_t i = 0; i < contacts; i++) {
    // ESP_LOGI(LOG_TAG,"C%d: #%d %d,%d s:%d", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
   printf("C%d: #%d %d,%d s:%d\n", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
  }
}


esp_err_t gt911_init(const touch_panel_config_t *config)
{    
    esp_err_t ret = ESP_OK;
    // Take chip some time to start
    msSleep(300);
    bool result = reset();
    msSleep(200);

    dumpRegID(); 
    // int8_t contacts = readInput(points);
    return ret;
}

esp_err_t gt911_deinit(void)
{
    //i2c_bus_device_delete(&g_dev.i2c_dev);
    memset(&g_dev, 0, sizeof(gt911_dev_t));
    return ESP_OK;
}

esp_err_t gt911_set_direction(touch_panel_dir_t dir)
{
    if (TOUCH_DIR_MAX < dir) {
        dir >>= 5;
    }
    g_dev.direction = dir;
    return ESP_OK;
}

static void gt911_apply_rotate(uint16_t *x, uint16_t *y)
{
    uint16_t _x = *x;
    uint16_t _y = *y;

    switch (g_dev.direction) {
    case TOUCH_DIR_LRTB:
        *x = _x;
        *y = _y;
        break;
    case TOUCH_DIR_LRBT:
        *x = _x;
        *y = g_dev.height - _y;
        break;
    case TOUCH_DIR_RLTB:
        *x = g_dev.width - _x;
        *y = _y;
        break;
    case TOUCH_DIR_RLBT:
        *x = g_dev.width - _x;
        *y = g_dev.height - _y;
        break;
    case TOUCH_DIR_TBLR:
        *x = _y;
        *y = _x;
        break;
    case TOUCH_DIR_BTLR:
        *x = _y;
        *y = g_dev.width - _x;
        break;
    case TOUCH_DIR_TBRL:
        *x = g_dev.height - _y;
        *y = _x;
        break;
    case TOUCH_DIR_BTRL:
        *x = g_dev.height - _y;
        *y = g_dev.width - _x;
        break;

    default:
        break;
    }
}

esp_err_t gt911_sample(touch_panel_points_t *info)
{
    gt911_dev_t *dev = &g_dev;
    
    int8_t contacts = readInput(points);
    info->point_num = contacts;
    if (info->point_num > 0 && info->point_num <= TOUCH_MAX_POINT_NUMBER) 
    {
        // handleTouch(contacts, (struct GTPoint *)points);
        struct GTPoint *Gpoints = (struct GTPoint *)points;
        for (size_t i = 0; i < info->point_num; i++)
        {
            info->curx[i] = Gpoints[i].x;
            info->cury[i] = Gpoints[i].y;
        }
        info->event = TOUCH_EVT_PRESS;
        gdx_write(GTP_READ_COORD_ADDR, 0);
    }
    else
    {
        info->curx[0] = 0;
        info->cury[0] = 0;
        info->event = TOUCH_EVT_RELEASE;
    }    
   
    return ESP_OK;
}

static esp_err_t gt911_calibration_run(const scr_driver_t *screen, bool recalibrate)
{
    (void)screen;
    (void)recalibrate;
    /**
     * The capacitive touch screen does not need to be calibrated
     */
    return ESP_OK;
}

