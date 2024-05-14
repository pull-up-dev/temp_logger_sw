# temp_logger_sw

## Description of the project

Simple application for temp_logger board: read temperature from TI TMP102 sensor.


## Installation

- Arduino IDE v2
    - ESP32 board platform: Tools -> Board ... -> Board Manager. Search "esp32" and install "esp32" by Espressif Systems.
- CP210x Unversal Windows Driver: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads. 
    In the downloaded Zip file, right-click on silabser.inf -> Install.


## Build

- Select board type "ESP32C3 Dev Module", and the COM port which corresponds to your board
- In the Serial Monitor of the Arduino IDE, select a baud rate of 115200.