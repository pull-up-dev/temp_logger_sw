/**
Simple example application to read data from the TMP102 temperature sensor with an ESP32-C3.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 7

#define SD_CS_PIN 5
#define SD_MOSI_PIN 4
#define SD_MISO_PIN 3
#define SD_SCK_PIN 2

#define BOOT_BT_PIN 9

#define TMP102_REG_TEMPERATURE 0x00
#define TMP102_REG_CONFIG 0x01
#define TMP102_REG_TLOW 0x02
#define TMP102_REG_THIGH 0x03

const unsigned long TEMP_READ_INTERVAL = 1000U;
const unsigned long BUTTON_DEBOUNCE_TIME = 50U;

const uint16_t TMP102_I2C_ADDR = 0x0048;  // 7-bit address

File temp_log_file;
SPIClass spiSD(FSPI);

void setup() {
  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin(); // Start I2C bus
  Serial.begin(115200);

  pinMode(BOOT_BT_PIN, INPUT_PULLUP);

  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  pinMode(spiSD.pinSS(), OUTPUT);

  if (!SD.begin(SD_CS_PIN, spiSD)) {
    Serial.println("SD initialization failed!");
    while (1) {
      // Do nothing
    }
  }
}

void loop() {
  static unsigned long lastReadTime = 0U;
  static uint32_t time_index = 0U;

  static bool writeFileEnable = false;
  static int prevBootButtonState = HIGH; // HIGH or LOW
  static unsigned long lastBootButtonStateChange = 0U;

  unsigned long currentTime = millis();

  int currentBootButtonState = digitalRead(BOOT_BT_PIN);
  if ((currentTime - lastBootButtonStateChange) >= BUTTON_DEBOUNCE_TIME) {
    if (currentBootButtonState != prevBootButtonState) {
        if (currentBootButtonState == HIGH) { // If button released
          // Start / stop temperature recording
          if (!writeFileEnable) {
            writeFileEnable = true;

            temp_log_file = SD.open("/temp.txt", FILE_WRITE, true);
            if (temp_log_file) {
              writeFileEnable = true;
              
              // Write legend to file
              if (!temp_log_file.println("Timestamp (s); Temperature (°C)")) {
                Serial.println("Failed to write legend to file");
              }

              Serial.println("Start recording");
            } else {
              Serial.println("Failed to create file 'temp.txt', NOT starting recording");
            }
          } else {
            writeFileEnable = false;
            
            temp_log_file.close();

            Serial.println("Stop recording");
          }
        }

        // Save current button status
        lastBootButtonStateChange = currentTime;
        prevBootButtonState = currentBootButtonState;
        // Serial.print("New button state: ");
        // Serial.println((prevBootButtonState == HIGH) ? "HIGH" : "LOW");
    }
  }

  if ((currentTime - lastReadTime) >= TEMP_READ_INTERVAL) {
    // Read temperature
    Wire.beginTransmission(TMP102_I2C_ADDR);
    Wire.write(TMP102_REG_TEMPERATURE);  // Temperature register (read-only)
    Wire.endTransmission();
    Wire.requestFrom(TMP102_I2C_ADDR, (uint8_t)2);  // Read 2 bytes
    if (Wire.available()) {
      uint16_t temp_uint = (Wire.read() << 4);  // Byte 1: bits 11 .. 4
      temp_uint |= (Wire.read() >> 4);          // Byte 2: bits 3 .. 0
      if ((temp_uint & 0x800) > 0U) {           // If sign bit is set, propagate it to 4 lsbits
        temp_uint |= 0xF000;
      }
      float temp_C = (float)((int16_t)temp_uint) * 0.0625f;  // 2-steps conversion: unsigned 16-bit integer -> signed 16-bit integer -> float, and multiplication by lsb

      // Print temperature to serial
      Serial.print(temp_C);
      Serial.println(" °C");

      // Write time and temperature to file
      if (writeFileEnable) {
        if ((!temp_log_file.print(time_index)) || (!temp_log_file.print(";")) || (!temp_log_file.println(temp_C))) {
          Serial.println("Failed to write to file");
        }
      }
    } else {
      Serial.println("no data");
    }

    time_index++;
    lastReadTime = currentTime;
  }

  delay(10);
}

void spiCommand(SPIClass& spi, byte data) {
  //use it as you would the regular arduino SPI API
  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(spi.pinSS(), LOW);  //pull SS slow to prep other end for transfer
  spi.transfer(data);
  digitalWrite(spi.pinSS(), HIGH);  //pull ss high to signify end of data transfer
  spi.endTransaction();
}
