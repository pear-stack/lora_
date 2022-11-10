#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <HX711.h>
#define BATTERY_PIN 35 // battery level measurement pin

void hx711_setup(const int32_t DOUT_PIN, const int32_t SCK_PIN, HX711 *scale, float calibration_factor);
void print_weight();
void get_weight_data(HX711 *scale, float *weight);

void print_bme280_data(Adafruit_BME280 *bme); // print bme280 data
void check_bme280_status(bool status, Adafruit_BME280 *bme); //check bme sensor
void get_bme280_data(Adafruit_BME280 *bme0, bool bme0_status, Adafruit_BME280 *bme1, bool bme1_status, float *temp0, float *temp1, float *hum0, float *hum1, float *pressure0);

void get_battery_voltage(float *vBat);

#endif