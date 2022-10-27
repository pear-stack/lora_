#include <Arduino.h>
#include "hx711_module.h"

/**********Sensor Pinout*********
---------------------------------
|   hx711     |    ESP12-E      |
---------------------------------
|   VDD         |     3V        |
|   VCC         |     -         |
|   DAT         |     D6        |
|   CLK         |     D5        |
|   GND         |     GND       |
---------------------------------
********************************/

void hx711_setup(const int32_t DOUT_PIN, const int32_t SCK_PIN, HX711 *scale, float calibration_factor){
  // pins: Dout = 12 (D6) , Clk = 14 (D5):
  // sensor init
  scale->begin(DOUT_PIN, SCK_PIN);
  // scale reset
  scale->set_scale();
  // set scale with calibration_fator 
  scale->set_scale(calibration_factor); 
};


void print_weight(HX711 *scale){
    Serial.println("weight: ");
    Serial.println(scale->get_units(), 1);
    
}