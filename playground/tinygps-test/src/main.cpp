#include <Arduino.h>
#include <TinyGPSPlus.h>

TinyGPSPlus gps;

HardwareSerial gpsSerial(2);

void getgps();

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
}

void getgps() {
    // Get location
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());

        if (gps.location.isUpdated()){
            Serial.print("Latitude= "); 
            Serial.print(gps.location.lat(), 2);
            Serial.print(" Longitude= "); 
            Serial.println(gps.location.lng(), 2);

            // time - in 24 hr format
            // Hour (0-23) (u8) 
            Serial.print(gps.time.hour());Serial.print(":");
            // Minute (0-59) (u8)
            Serial.print(gps.time.minute());Serial.print(":");
            // Second (0-59) (u8)
            Serial.print(gps.time.second());
            Serial.println();
            // 100ths of a second (0-99) (u8)
        } 
    }
}

void loop() {
//     while (gpsSerial.available() > 0){
//     // get the byte data from the GPS
//     byte gpsData = gpsSerial.read();
//     Serial.write(gpsData);
//   }

  getgps();
}