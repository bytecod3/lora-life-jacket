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

int water_raw_val = 0; // value for storin water level
int currentBtnState, previousBtnState;
unsigned int lastDebounceTime = 0;
unsigned int debounceDelay = 100;

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
int readWaterLevel();
uint8_t readPanicButton();
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
    pinMode(WATER_LEVEL_POWER_PIN, OUTPUT);
    digitalWrite(WATER_LEVEL_POWER_PIN, LOW);
    debugln("Water level sensor init OK!");
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
    debugln("Panic button init OK!");
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
 * This is debounced via software
 * If it is pressed, send a message via LORA to base station
 * Light up some LED
 * 
 */
uint8_t readPanicButton() {
    currentBtnState = digitalRead(BUTTON_PIN);
    if(currentBtnState != previousBtnState) {
        // get the time if the change, every time a button state changes
        lastDebounceTime = millis(); 
    }

    if( (millis() - lastDebounceTime) > debounceDelay) {
        return digitalRead(currentBtnState);
    }
    
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

/**
 * @brief read water level
 * 
 */
int readWaterLevel() {
    digitalWrite(WATER_LEVEL_POWER_PIN, HIGH); // turn the sensor ON
    delay(20);
    water_raw_val = analogRead(WATER_LEVEL_ADC_PIN);
    digitalWrite(WATER_LEVEL_ADC_PIN, LOW);
    return water_raw_val;
}

/**
 * @brief Initialize the hardware
 * 
 */
void initHW() {
    Serial.begin(BAUDRATE);

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