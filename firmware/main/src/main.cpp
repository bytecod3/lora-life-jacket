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

int jacket_id_ = 1001; // unique ID for this jacket

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

char lora_msg[128];
double latitude;
double longitude;
uint8_t day=0, month=0, hr=0, mint=0, sec=0;
uint16_t year=0;

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
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());

        if (gps.location.isUpdated()){ 
            // Get location
            debug("Location: ");
            if(gps.location.isValid()) {
                latitude = gps.location.lat();
                Serial.print(gps.location.lat(), 2);
                debug(F(","));

                longitude = gps.location.lng();
                Serial.print(gps.location.lng(), 2);

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

                hr = gps.time.hour();
                if (hr < 10) debug(F("0"));
                debug(hr);

                mint = gps.time.minute();
                if (mint < 10) debug(F("0"));
                debug(mint);

                sec = gps.time.second();
                if (sec < 10) debug(F("0"));
                debug(sec);

            } else {
                debug(F("INVALID"));
            }

            debugln();
        }
    }

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

    // read and check water level
    // check for water level
    readGPS();

    // compose LORA message
    sprintf(lora_msg, "%d,SOS,%.2f,%.2f,%d,%d,%d,%d,%d,%d\n",
            jacket_id_,
            latitude,
            longitude,
            hr,
            mint,
            sec,
            day,
            month,
            year
            );

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
            debugln(lora_msg);
            // send lora message
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