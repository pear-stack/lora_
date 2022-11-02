#include <sensors.h>

/**********Sensor Pinout*********
---------------------------------
|   BME280     |       ESP      |
---------------------------------
|   CS         |     -          |
|   ADDR/MISO  |     3V         |
|   SCL/SCK    |     D1         |
|   SDA/MOSI   |     D2         |
|   GND        |     GND        |
|   VCC        |     VCC        |
---------------------------------
********************************/

void check_bme280_status(bool status, Adafruit_BME280 *bme)
{
  if(!status)
  {
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
  else
  {
    Serial.println("BME280 sensor: OK");
  }
}

void print_bme280_data(Adafruit_BME280 *bme) 
{
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

void get_bme280_data(bool one_sensor, Adafruit_BME280 *bme0, bool bme0_status, Adafruit_BME280 *bme1, bool bme1_status, float *temp0, float *temp1, float *hum0, float *hum1, float *pressure0)
{
  if(one_sensor)
    bme1_status = 1;
  if(!bme0_status || !bme1_status)
  {
    *temp0 = 0.0;
    *hum0 = 0.0;
    *pressure0 = 0.0;
    *temp1 = 0.0;
    *hum1 = 0.0;
  }
  else
  {
    *temp0 = bme0->readTemperature();
    *hum0 = bme0->readHumidity();
    *pressure0 = bme0->readPressure()/100;
    *temp1 = bme1->readTemperature();
    *hum1 = bme1->readHumidity();
  }
  delay(100);
}

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

void hx711_setup(const int32_t DOUT_PIN, const int32_t SCK_PIN, HX711 *scale, float calibration_factor)
{
  // pins: Dout = 12 (D6) , Clk = 14 (D5):
  // sensor init
  scale->begin(DOUT_PIN, SCK_PIN);
  // scale reset
  scale->set_scale();
  // set scale with calibration_fator 
  scale->set_scale(calibration_factor); 
};


void print_weight(HX711 *scale)
{
  Serial.println("weight: ");
  Serial.println(scale->get_units(), 1);
}

void get_weight_data(HX711 *scale, float *weight)
{
  *weight = scale->get_units();
}

void get_battery_voltage(float *vBat) {
  *vBat = analogRead(BATTERY_PIN) * (3.3 / 1024.0);
  Serial.print("Battery voltage: ");
  Serial.print(*vBat);
  Serial.println("V");  
}

