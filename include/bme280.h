#pragma once

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

void print_bme280_data(Adafruit_BME280 *bme); // print bme280 data
void check_bme280_status(bool status, Adafruit_BME280 *bme); //check bme sensor