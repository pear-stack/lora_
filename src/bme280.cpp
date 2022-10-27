#include <Arduino.h>
#include "bme280.h"

/**********Sensor Pinout*********
---------------------------------
|   BME280     |    ESP12-E     |
---------------------------------
|   CS         |     -          |
|   ADDR/MISO  |     3V         |
|   SCL/SCK    |     D1         |
|   SDA/MOSI   |     D2         |
|   GND        |     GND        |
|   VCC        |     VCC        |
---------------------------------
********************************/

void check_bme280_status(bool status, Adafruit_BME280 *bme){
  if(!status){
    Serial.println(F("BME280 test"));
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme->sensorID(),16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while(1) 
      delay(10);
  }
  else{
      Serial.println("BME280 sensor: OK");
  }
}

void print_bme280_data(Adafruit_BME280 *bme) {
    Serial.print("Temperature = ");
    Serial.print(bme->readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme->readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme->readHumidity());
    Serial.println(" %");

    Serial.println();
}