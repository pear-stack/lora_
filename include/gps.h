#ifndef _GPS_H_
#define _GPS_H_

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define GPS_TX 34
#define GPS_RX 12

class GPS
{
    public:
        void init();
        bool checkGpsFix();
        void getLatLon(double* lat, double* lon, double *alt, int *sats);
        void encode();

    private:
        char t[32]; // used to sprintf for Serial output
        TinyGPSPlus tGps;
};

#endif