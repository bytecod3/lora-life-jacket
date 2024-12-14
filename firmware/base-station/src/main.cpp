/*
  * Author - Edwin Mwiti
  * July 20 2024
*/
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <MFRC522.h>

#define DEBUG 1

#if DEBUG
#define debug(x)   Serial.print(x);
#define debugln(x)  Serial.println(x);
#define debugf(x, y) Serial.printf(x, y)

#else 
#define debug(x) 
#define debugln(x)  
#endif // END DEBUG

// RFID reader Pins
#define RFID_RST    12
#define RFID_CS     5
const int cs_pin = 4; 
const int reset_pin = 2;
const int irq_pin = 22; // DIO0 on HOPE RF LORA MODULE - pin must have hardware interrupt


MFRC522 rfid(RFID_CS, RFID_RST);
MFRC522::MIFARE_Key key;
// Init array that will store new NUID 
byte nuidPICC[4];

void initRFID();
void readRFID();
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

void setup() {
  Serial.begin(115200);
 
  LoRa.setPins(cs_pin, reset_pin, irq_pin);
  Serial.println("LoRa Receiver");
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
  } else {
    Serial.println("LORA STARTED OK!");
  }

  // initialize RFID
  initRFID();
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    
    Serial.println();

  }


  readRFID();

}