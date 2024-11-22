#include <Arduino.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include "defs.h"

/**
 * Object instances
 */
// create an instance of the hardwareSerial class for serial2
HardwareSerial gpsSerial(2);

/**
 * Function prototypes
 * 
 */
void initLORA();
void initGPS();
void initWaterLevelSensor();
void initLEDs();
void initRFID();
void readGPS();
void readWaterLevel();
void readPanicButton();
void activateLEDs();
void sendLORAMsg(char* );
void readRFID();
void initHW();

/**
 * Functions implementation
 */

/**
 * @brief Initialize the LORA transceivers
 * @param none
 */
void initLORA() {
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO);
    if(!LoRa.begin(LORA_FREQUENCY)) {
        debugln("Lora failed to start");
    } else {
        debugln("Lora started OK!");
    }
}

/**
 * @brief Initialize water level sensor
 * We will power the water level sensor only when taking readings 
 */
void initWaterLevelSensor() {
    
}

/**
 * @brief Initialize LEDs
 * 
 */ 
void initLEDs() {
    pinMode(PANIC_LED, OUTPUT);
    debugln("LEDs init OK!");
}

/**
 * @brief Init panic button
 */
void initPanicButton() {
    pinMode(BUTTON_PIN, INPUT);
}

/**
 * @brief Initialize the GPS module
 * 
 */
void initGPS() {
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);\
    debugln("GPS serial2 started at 9600 BAUD.");
}


/**
 * @brief Send message via LORA
 */
void sendLORAMsg(char* msg) {
    debugln("Sending LORA message");

    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    delay(LORA_DELAY);
}

/**
 * 
 */

/**
 * @brief Read the panic button attached to the life jacket
 * If it is pressed, send a message via LORA to base station
 * Light up some LED
 * 
 */
void readPanicButton() {

}

/**
 * @brief Read GPS 
 */
void readGPS() {
    while(gpsSerial.available() > 0) {
        // get byte data from the GPS
        char gpsData = gpsSerial.read();
        debug(gpsData);
    }

    delay(500);
}

void initHW() {
    Serial.begin(BAUDRATE);

    // for the water level sensor
    pinMode(WATER_LEVEL_POWER_PIN, OUTPUT);
    debugln("Water level sensor init OK!");

    initLORA();
    initGPS();
    initWaterLevelSensor();
    initLEDs();
    initPanicButton();
    initRFID();

    

}

void setup() {
    void initHW();

}

void loop() {
    
}