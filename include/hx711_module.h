#pragma once

#include "HX711.h"

void hx711_setup(const int32_t DOUT_PIN, const int32_t SCK_PIN, HX711 *scale, float calibration_factor);
void print_weight();

