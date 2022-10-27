#ifndef __CREDITIONALS_H__
#define __CREDITIONALS_H__

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>

//TTN working keys
// This EUI must be in little-endian format, so least-significant-byte first. 
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// This should also be in little endian format.
static const u1_t PROGMEM DEVEUI[8]={ 0x48, 0xE3, 0x83, 0xFC, 0x65, 0x6D, 0xA3, 0xB0 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// This key should be in big endian format.
static const u1_t PROGMEM APPKEY[16] = { 0xb0, 0x2b, 0x98, 0x59, 0xbb, 0xc7, 0x1b, 0xa2, 0xa6, 0x1b, 0x40, 0x19, 0x0a, 0xb9, 0x2a, 0xb0 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

/*Chirpstack working keys
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ 0xb0, 0xa3, 0x6d, 0x65, 0xfc, 0x83, 0xe3, 0x48 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0xb0, 0x2b, 0x98, 0x59, 0xbb, 0xc7, 0x1b, 0xa2, 0xa6, 0x1b, 0x40, 0x19, 0x0a, 0xb9, 0x2a, 0xb0 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
*/

#endif