#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include "defs.h"

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
 * @brief Initialize LEDs
 * 
 */ 
void initLEDs() {
    pinMode(PANIC_LED, OUTPUT);
}

/**
 * @brief Initialize the GPS module
 * 
 */
void initGPS() {
    
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

void initHW() {
    Serial.begin(BAUDRATE);
    initLORA();
    initGPS();
    initWaterLevelSensor();
    initLEDs();
    initRFID();

}

void setup() {
    void initHW();

}

void loop() {
    
}