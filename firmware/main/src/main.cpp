/**
 * This is the firmware used for the life jacket onboard device
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <MFRC522.h>
#include "defs.h"

/**
 * Object instances
 */
// create an instance of the hardwareSerial class for serial2
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
MFRC522 rfid(RFID_CS, RFID_RST);
MFRC522::MIFARE_Key key; 

// Init array that will store new NUID 
byte nuidPICC[4];

int water_raw_val = 0; // value for storing water level

// button press variables
#define DEBOUNCE_DELAY 50
int lastSteadyState = LOW; // previous steady state from the input pin
int lastFlickerableState = LOW; // previous flickerable state from the input pin
int currentState; // the current reading from the input pin
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
int buttonPressed = 0;


char lora_msg[100];
double latitude;
double longitude;
uint8_t day, month, year;

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
int readPanicButton();
void activateLEDs();
void sendLORAMsg(char* );
void readRFID();
void initHW();
void printHex(byte *buffer, byte bufferSize);
void printDec(byte *buffer, byte bufferSize);


/**
 * @brief Initialize RFID tags 
 * 
 */
void initRFID() {
    SPI.begin();
    rfid.PCD_Init();
    delay(10);

    rfid.PCD_DumpVersionToSerial(); // show card reader details 

    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }

    debugln(F("This code scan the MIFARE Classsic NUID."));
    Serial.print(F("Using the following key:"));
    printHex(key.keyByte, MFRC522::MF_KEY_SIZE);
    
    debugln(F("Scan PICC to see UID, SAK, type, and data blocks..."));

}

/**
 * Functions implementation
 */

/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}

/**
 * @brief Initialize the LORA transceivers
 * @param none
 */
void initLORA() {
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO);
    if(!LoRa.begin(LORA_FREQUENCY)) {
        debugln("Lora failed to start");
    } else {
        debugln("[+]Lora started OK!");
    }
}

/**
 * @brief Initialize water level sensor
 * We will power the water level sensor only when taking readings 
 */
void initWaterLevelSensor() {
    pinMode(WATER_LEVEL_PWR, OUTPUT);
    digitalWrite(WATER_LEVEL_PWR, LOW);
    debugln("[+]Water level sensor init OK!");
}

/**
 * @brief Initialize LEDs
 * 
 */ 
void initLEDs() {
    pinMode(LED1, OUTPUT);
    debugln("[+]LEDs init OK!");
}

/**
 * @brief Init panic button
 */
void initPanicButton() {
    pinMode(PANIC_BUTTON, INPUT);
    debugln("[+]Panic button init OK!");
}

/**
 * @brief Initialize the GPS module
 * 
 */
void initGPS() {
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);\
    debugln("[+]GPS serial2 started at 9600 BAUD.");
}


/**
 * @brief Send message via LORA
 */
void sendLORAMsg(char* msg) {
    debugln("[...]Sending LORA message");

    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    delay(LORA_DELAY);
}

/**
 * @brief Read the panic button attached to the life jacket
 * This is debounced via software
 * If it is pressed, send a message via LORA to base station
 * Light up some LED
 * 
 */
 int readPanicButton() {
    // read the state of the button
    int r;
    currentState = digitalRead(PANIC_BUTTON);
    if(currentState != lastFlickerableState) {
        lastDebounceTime = millis();
        lastFlickerableState = currentState;
    }
    if( (millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // if the button state has changed
        if ( (lastSteadyState == HIGH) && (currentState == LOW) ) {
            // button pressed
            r = 1;
        } else if( (lastSteadyState == LOW) && (currentState == HIGH) ) {
            // button released
            r = 0;
        }
        // save the last steady state
        lastSteadyState = currentState;
    }

    return r;
 }

/**
 * @brief Read the RFID reader for card presence
 * 
 */
void readRFID() {
    // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
	if ( !rfid.PICC_IsNewCardPresent()) {
		return;
	}

	// Select one of the cards
	if ( !rfid.PICC_ReadCardSerial()) {
		return;
	}

    // check the tag being scanned
	// Dump debug info about the card; PICC_HaltA() is automatically called
	rfid.PICC_DumpToSerial(&(rfid.uid));

    Serial.print(F("[+]PICC type: "));
    MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
    Serial.println(rfid.PICC_GetTypeName(piccType));

    // Check is the PICC of Classic MIFARE type
    if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI && 
        piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
        piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
        Serial.println(F("[-]Your tag is not of type MIFARE Classic."));
        return;
    }

    if (rfid.uid.uidByte[0] != nuidPICC[0] || 
        rfid.uid.uidByte[1] != nuidPICC[1] || 
        rfid.uid.uidByte[2] != nuidPICC[2] || 
        rfid.uid.uidByte[3] != nuidPICC[3] ) {
            Serial.println(F("[+]A new card has been detected."));

            // Store NUID into nuidPICC array
            for (byte i = 0; i < 4; i++) {
                nuidPICC[i] = rfid.uid.uidByte[i];
            }
    
            Serial.println(F("[+]The NUID tag is:"));
            Serial.print(F("In hex: "));
            printHex(rfid.uid.uidByte, rfid.uid.size);
            Serial.println();
            Serial.print(F("In dec: "));
            printDec(rfid.uid.uidByte, rfid.uid.size);
            Serial.println();

    } else {
        Serial.println(F("[+]Card read previously."));
    }

    // Halt PICC
    rfid.PICC_HaltA();

    // Stop encryption on PCD
    rfid.PCD_StopCrypto1();

}

/**
 * @brief Read GPS 
 */
void readGPS() {

    // Get location
    debugln("Location: ");
    if(gps.location.isValid()) {
        latitude = gps.location.lat();
        Serial.print(latitude, 6);
        debug(F(","));

        longitude = gps.location.lng();
        Serial.print(longitude, 6);

    } else {
        debug(F("INVALID"));
    }

    // Get time and date
    debug(F("Date/time: "));
    if(gps.date.isValid()) {
        month = gps.date.month();
        debug(month);
        debug(F("/"));

        day = gps.date.day();
        debug(day);
        debug(F("/"));

        year = gps.date.year();
        debug(year);
    } else {
        debug(F("INVALID"));
    }

    // time 
    debug(F(" "));
    if (gps.time.isValid()) {

        if (gps.time.hour() < 10) debug(F("0"));
        debug(gps.time.hour());
        debug(F(":"));

        if (gps.time.minute() < 10) debug(F("0"));
        debug(gps.time.minute());
        debug(F(":"));

        if (gps.time.second() < 10) debug(F("0"));
        debug(gps.time.second());
        debug(F("."));

        if (gps.time.centisecond() < 10) debug(F("0"));
        debug(gps.time.centisecond());

    } else
    {
        debug(F("INVALID"));
    }

    debugln();

}

/**
 * @brief Read water level
 * 
 */
int readWaterLevel() {
    digitalWrite(WATER_LEVEL_PWR, HIGH); // turn the sensor ON
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
    debugln("Initializing hardware");
    initHW();
    debugln();

}

void loop() {

    // read RFID
//    readRFID();

    // read and check water level
    // check for water level



    // read and check panic button
    currentState = digitalRead(PANIC_BUTTON);
    if(currentState != lastFlickerableState) {
        lastDebounceTime = millis();
        lastFlickerableState = currentState;
    }
    if( (millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // if the button state has changed
        if ( (lastSteadyState == HIGH) && (currentState == LOW) ) {
            // button pressed
            // compose LORA message for transmission
            sprintf(lora_msg, "SOS\r\n");

            // send lora message
            Serial.print("Panic button pressed. Sending: ");
            Serial.print(lora_msg);

            LoRa.beginPacket();
            LoRa.print(lora_msg);
            LoRa.endPacket();
            delay(LORA_DELAY);

        } else if( (lastSteadyState == LOW) && (currentState == HIGH) ) {
            // button released

        }
        // save the last steady state
        lastSteadyState = currentState;
    }


}