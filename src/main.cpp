#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <creditionals.h>
#include <Wire.h>
#include <WiFi.h>
#include <gps.h>
#include <CayenneLPP.h>
#include <bme280.h>
#include <hx711_module.h>

#define DEBUG 1 // for real use comment this out
#define BUILTIN_LED 14 // T-Beam blue LED, see: http://tinymicros.com/wiki/TTGO_T-Beam
#define BATTERY_PIN 35 // battery level measurement pin, here is the voltage divider connected

CayenneLPP lpp(51); // here we will construct Cayenne Low Power Payload (LPP) - see https://community.mydevices.com/t/cayenne-lpp-2-0/7510
GPS gps; // class that is encapsulating additional GPS functionality
#ifdef HX711_ON
HX711 scale;
#endif
Adafruit_BME280 bme0;
//Adafruit_BME280 bme1;

double lat, lon, alt; // GPS data are saved here: Latitude, Longitude, Altitude
int sats; // GPS satellite count
char s[32]; // used to sprintf for Serial output
float vBat; // battery voltage
long nextPacketTime;

float temp0, hum0, pressure0; // BME280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
bool bme0_status; // status after reading from BME280

//float temp1, hum1, pressure1; // BME280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
//bool bme1_status; // status after reading from BME280

void get_sensors_data(void); // declaration for function below
static osjob_t sendjob;

// wait this many seconds when no GPS fix is received to retry
const unsigned int GPS_FIX_RETRY_DELAY = 3; 

// Schedule TX every this many seconds.
const unsigned TX_INTERVAL = 5;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

void do_send(osjob_t* j);


void get_battery_voltage() {
  // we've set 10-bit ADC resolution 2^10=1024 and voltage divider makes it half of maximum readable value (which is 3.3V)
  vBat = analogRead(BATTERY_PIN) * (3.3 / 1024.0);
  Serial.print("Battery voltage: ");
  Serial.print(vBat);
  Serial.println("V");  
}

void get_sensors_data(void){
    //TODO: copy sensor handling
    if(!bme0_status)
    {
        temp0 = 0.0;
        hum0 = 0.0;
        pressure0 = 0.0;
    }
    else
    {
        print_bme280_data(&bme0);
        temp0 = bme0.readTemperature();
        hum0 = bme0.readHumidity();
        pressure0 = bme0.readPressure();
    }
    delay(100);
}


void print_hex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    #ifdef DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              #ifdef DEBUG
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                print_hex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      print_hex2(nwkKey[i]);
              }
              Serial.println();
              #endif
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	        // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            #ifdef DEBUG
            digitalWrite(BUILTIN_LED, LOW);
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            #endif
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            #ifdef DEBUG
            Serial.printf("Next lora packet schedudled in %d seconds.\n", TX_INTERVAL);
            #endif
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            #ifdef DEBUG
            Serial.println(F("EV_RXCOMPLETE"));
            #endif
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    get_battery_voltage();
    get_sensors_data();
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else 
    {
        // Prepare upstream data transmission at the next possible time.
        if (gps.checkGpsFix())
        {
            // Prepare upstream data transmission at the next possible time.
            gps.getLatLon(&lat, &lon, &alt, &sats);

            // we have all the data that we need, let's construct LPP packet for Cayenne
            lpp.reset();
            lpp.addGPS(1, 0xee, 0xee, 0xee);
            lpp.addTemperature(2, 0xff);
            lpp.addRelativeHumidity(3, 0xff);
            lpp.addBarometricPressure(4, 0xff);
            lpp.addAnalogInput(5, 0xaa);
            // optional: send current speed, satellite count, altitude from barometric sensor and battery voltage
            //lpp.addAnalogInput(6, kmph);
            lpp.addAnalogInput(7, sats);
            //lpp.addAnalogInput(8, alt_barometric);
            // read LPP packet bytes, write them to FIFO buffer of the LoRa module, queue packet to send to TTN
            LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
            
            Serial.print(lpp.getSize());
            Serial.println(F(" bytes long LPP packet queued."));
            digitalWrite(BUILTIN_LED, HIGH);
        }
        else
        {
            // try again in a few 'GPS_FIX_RETRY_DELAY' seconds...
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(GPS_FIX_RETRY_DELAY), do_send);
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
//*******************************SERIAL********************************// 
    Serial.begin(9600);
    Serial.println(F("Starting"));
//*******************************BATTERY********************************// 
    adcAttachPin(BATTERY_PIN);
    // Default of 12 is not very linear. Recommended 10 or 11 depending on needed resolution.
    analogReadResolution(10); 
//*******************************BT/WIFI********************************// 
    WiFi.mode(WIFI_OFF);
    btStop();
//********************************GPS**********************************// 
    gps.init();
//*******************************BME280********************************// 
    // address: 0x77 when ADDR PIN = HIGH, 0x76 when ADDR PIN = LOW
    // default I2C pins ESP12-E: SDA = GPIO 4 (D2), SCL = GPIO 5 (D1)
    // sensor init
    bme0_status = bme0.begin(0x76, &Wire);
    //bme1_status = bme1.begin(0x77, &Wire);
    // sensor test  
    check_bme280_status(bme0_status, &bme0);
    //check_bme280_status(&bme1_status, &bme1);
//*******************************HX711*********************************//
    #ifdef HX711_ON
    // HX711 setup
    // pins: Dout = 12 (D6) , Clk = 14 (D5):
    // calibration factor = 48660;
    hx711_setup(12, 14, &scale, 48650);
    #endif
//**********************************OS***********************************// 
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
//***********************************************************************//
}

void loop() {
    os_runloop_once();
}