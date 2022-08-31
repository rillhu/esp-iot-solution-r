@echo off

REM BAT
REM Copyright 2022 MicroEJ Corp. All rights reserved.
REM Use of this source code is governed by a BSD-style license that can be found with this software.

SET CURRENT_DIRECTORY=%CD%
ECHO CURRENT_DIRECTORY=%CURRENT_DIRECTORY%

CD "%~dp0.."
SET ESP_PROJECT_DIR=%CD%
ECHO ESP_PROJECT_DIR=%ESP_PROJECT_DIR%

REM Set IDF tools path.
REM SET IDF_TOOLS_PATH=%USERPROFILE%\.espressif
REM ECHO IDF_TOOLS_PATH=%IDF_TOOLS_PATH%

REM Set serial port. Set a value (e.g. COM4).
SET ESPPORT=COM24
ECHO ESPPORT=%ESPPORT%
REM Set ESP_IDF_PATH.
SET ESP_IDF_PATH=C:\ZCodes\SDKs\esp-idf-v4.3.1
ECHO ESP_IDF_PATH=%ESP_IDF_PATH%

SET IOT_SOLUTION_PATH=c:\ZCodes\Git\esp-iot-solution-feeeb6
ECHO IOT_SOLUTION_PATH=%IOT_SOLUTION_PATH%



CD "%ESP_IDF_PATH%"
CALL install.bat

REM Restore current directory
CD "%CURRENT_DIRECTORY%"
