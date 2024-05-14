/**
Simple example application to read data from the TMP102 temperature sensor with an ESP32-C3.
*/

#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 7

#define TMP102_REG_TEMPERATURE 0x00
#define TMP102_REG_CONFIG 0x01
#define TMP102_REG_TLOW 0x02
#define TMP102_REG_THIGH 0x03

const uint16_t TMP102_I2C_ADDR = 0x0048; // 7-bit address

int32_t temperature_C = 0;

void setup() {
  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin(); // Start I2C bus
  Serial.begin(115200);
}

void loop() {
  // Read temperature
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write(TMP102_REG_TEMPERATURE); // Temperature register (read-only)
  Wire.endTransmission();
  Wire.requestFrom(TMP102_I2C_ADDR, (uint8_t)2); // Read 2 bytes
  if (Wire.available()) {
    uint16_t temp_uint = (Wire.read() << 4);  // Byte 1: bits 11 .. 4
    temp_uint |= (Wire.read() >> 4);          // Byte 2: bits 3 .. 0
    if ((temp_uint & 0x800) > 0U) { // If sign bit is set, propagate it to 4 lsbits
      temp_uint |= 0xF000;
    }
    float temp_C = (float)((int16_t)temp_uint) * 0.0625f; // 2-steps conversion: unsigned 16-bit integer -> signed 16-bit integer -> float, and multiplication by lsb

    Serial.print(temp_C);
    Serial.println(" °C");
  } else {
    Serial.println("no data");
  }

  delay(1000);
}
