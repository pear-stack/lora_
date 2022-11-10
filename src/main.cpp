#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <creditionals.h>
#include <WiFi.h>
#include <gps.h>
#include <CayenneLPP.h>
#include <sensors.h>
#include <esp_sleep.h>

#define GPS_OFF
#define DEBUG 1 // for real use comment this out
#define SLEEP_BETWEEN_MESSAGES 1
#define SLEEP_TIME_MS 30000
#define SLEEP_DELAY_MS 1000

CayenneLPP lpp(28); // Cayenne Low Power Payload (LPP) 
#ifdef GPS_ON
GPS gps;
#endif
#ifdef BME0_ON 
Adafruit_BME280 bme0;
#endif
#ifdef BME1_ON
Adafruit_BME280 bme1;
#endif
#ifdef HX711_ON
HX711 scale;
#endif
// GPS data 
double lat, lon, alt;
int sats; 
// battery voltage
float vBat; 
// BME280 data 
float temp0, hum0, pressure0; 
float temp1, hum1, weight; 
// status after reading from BME280
bool bme0_status; 
bool bme1_status;

static osjob_t sendjob;
// wait this many seconds when no GPS fix is received to retry
const unsigned int GPS_FIX_RETRY_DELAY = 3000; 
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

void do_send(osjob_t* j);

void print_hex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void sleep_interrupt(uint8_t gpio, uint8_t mode) {
    esp_sleep_enable_ext0_wakeup((gpio_num_t) gpio, mode);
}

void sleep_interrupt_mask(uint64_t mask, uint8_t mode) {
    esp_sleep_enable_ext1_wakeup(mask, (esp_sleep_ext1_wakeup_mode_t) mode);
}

void sleep_millis(uint64_t ms) {
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_deep_sleep_start();
}

void sleep_seconds(uint32_t seconds) {
    esp_sleep_enable_timer_wakeup(seconds * 1000000);
    esp_deep_sleep_start();
}

void sleep_forever() {
    esp_deep_sleep_start();
}


void sleep() {
    #if SLEEP_BETWEEN_MESSAGES
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Sleeping in %d\n", (SLEEP_DELAY_MS / 1000));
        Serial.print(buffer);
        delay(SLEEP_DELAY_MS);
        sleep_millis(SLEEP_TIME_MS);
    #endif
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
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
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            sleep();
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
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
    get_battery_voltage(&vBat);
    //TODO: CHANGE bme0 to bme1 in 3rd argument
    #ifdef BME0_ON 
    get_bme280_data(&bme0, bme0_status, &bme0, bme0_status, &temp0, &temp1, &hum0, &hum1, &pressure0); 
    #endif
    #ifdef HX711_ON
    get_weight_data(&scale, &weight);
    #endif
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else 
    {
        #ifdef GPS_ON
        // Prepare upstream data transmission at the next possible time.
        if (gps.checkGpsFix())
        {
            // Prepare upstream data transmission at the next possible time.
            gps.getLatLon(&lat, &lon, &alt, &sats);
        #endif
            // Pack LPP packet
            lpp.reset();
        #ifdef GPS_ON
            lpp.addGPS(1, lat, lon, alt);
        #endif
            lpp.addTemperature(1, 0xff);
            lpp.addTemperature(2 ,0xff);
            lpp.addRelativeHumidity(1, 0xff);
            lpp.addRelativeHumidity(2, 0xff);
            lpp.addBarometricPressure(1, 0xff);
            lpp.addAnalogInput(1, vBat);
            lpp.addAnalogInput(2, 0xff); //hx711
            // read LPP packet bytes, write them to FIFO buffer of the LoRa module, queue packet to send 
            LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
            Serial.print(lpp.getSize());
            Serial.println(F(" bytes long LPP packet queued."));
        #ifdef GPS_ON
        }
        else
        {
           while(!gps.checkGpsFix()){
                delay(GPS_FIX_RETRY_DELAY);
           }
        }
        #endif
    }
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
    #ifdef GPS_ON
    gps.init();
    #endif
//*******************************BME280********************************// 
    // address: 0x77 when ADDR PIN = HIGH, 0x76 when ADDR PIN = LOW
    // default I2C pins ESP12-E: SDA = GPIO 4 (D2), SCL = GPIO 5 (D1)
    // sensor init
    #ifdef BME0_ON 
    bme0_status = bme0.begin(0x76, &Wire);
    #endif
    #ifdef BME1_ON
    bme1_status = bme1.begin(0x77, &Wire);
    #endif
    #ifdef BME0_ON 
    check_bme280_status(bme0_status, &bme0);
    bme0.setSampling(Adafruit_BME280::MODE_FORCED, // Force reading after delayTime
                     Adafruit_BME280::SAMPLING_X1, // Temperature sampling set to 1
                     Adafruit_BME280::SAMPLING_X1, // Pressure sampling set to 1
                     Adafruit_BME280::SAMPLING_X1, // Humidity sampling set to 1
                     Adafruit_BME280::FILTER_OFF   // Filter off - immediate 100% step response
                    );
     #endif
    #ifdef BME1_ON
    check_bme280_status(&bme1_status, &bme1);
    bme1.setSampling(Adafruit_BME280::MODE_FORCED, // Force reading after delayTime
                    Adafruit_BME280::SAMPLING_X1, // Temperature sampling set to 1
                    Adafruit_BME280::SAMPLING_X1, // Pressure sampling set to 1
                    Adafruit_BME280::SAMPLING_X1, // Humidity sampling set to 1
                    Adafruit_BME280::FILTER_OFF   // Filter off - immediate 100% step response
                    );
    #endif
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