#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

#define WIFI_SSID  "Gakibia-Unit3"
#define WIFI_PASSWORD  "password"

#define DEBUG 1

#if DEBUG
#define debug(x)   Serial.print(x);
#define debugln(x)  Serial.println(x);
#define debugf(x, y) Serial.printf(x, y);

#else
#define debug(x) ;
#define debugln(x) ;
#endif // END DEBUG

// RFID reader Pins
#define RFID_RST    12
#define RFID_CS     5
const int cs_pin = 4;
const int reset_pin = 2;
const int irq_pin = 22; // DIO0 on HOPE RF LORA MODULE - pin must have hardware interrupt
int LED = 26;

unsigned long previousBlink = 0;
int blink_interval = 200;
int ledState = 0;

MFRC522 rfid(RFID_CS, RFID_RST);
MFRC522::MIFARE_Key key;

// Init array that will store new NUID
byte nuidPICC[4];
//

String UID = "D7 C1 80 35";
int num_scans = 0;

// Server settings
const char* serverUrl = "http://192.168.1.120:3002/sos"; // Replace with your Node.js server address and endpoint

void initRFID();
void readRFID();
void printHex(byte *buffer, byte bufferSize);
void printDec(byte *buffer, byte bufferSize);
void sendToServer(String data);

/**
 * @brief Initialize RFID tags
 */
void initRFID() {
    SPI.begin();
    rfid.PCD_Init();
    delay(10);

    //rfid.PCD_DumpVersionToSerial(); // show card reader details

//    for (byte i = 0; i < 6; i++) {
//        key.keyByte[i] = 0xFF;
//    }

    // debugln(F("This code scans the MIFARE Classsic NUID."));
    // Serial.print(F("Using the following key:"));
    // printHex(key.keyByte, MFRC522::MF_KEY_SIZE);

    // debugln(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}


void  blinkLED() {
//    if((millis() - previousBlink) > blink_interval) {
//        ledState = !ledState;
//        digitalWrite(LED, ledState);
//        previousBlink = millis();
//    }

    digitalWrite(LED, HIGH);
    delay(blink_interval);
    digitalWrite(LED, LOW);
    delay(blink_interval);

}

/**
 * @brief Read the RFID reader for card presence
 */
void readRFID() {
    if (!rfid.PICC_IsNewCardPresent()) {
        return;
    }

    if (!rfid.PICC_ReadCardSerial()) {
        return;
    }

    //Serial.print("NUID tag is :");
    String ID = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
        ID.concat(String(rfid.uid.uidByte[i] < 0x10 ? " 0" : " "));
        ID.concat(String(rfid.uid.uidByte[i], HEX));
        delay(20);
    }
    ID.toUpperCase();

    if(ID.substring(1) == UID) {
        num_scans++;
    }

    blinkLED();

    if(num_scans == 1) {
        Serial.println("Jacket issued");
    }

    if(num_scans == 2) {
        num_scans = 0;
        Serial.println("Jacket returned");
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

/**
 * Send data to the server via HTTP POST
 */
void sendToServer(String data) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");

        // Create JSON payload
        StaticJsonDocument<200> doc;
        doc["data"] = data;

        String payload;
        serializeJson(doc, payload);

        // Send POST request
        int httpResponseCode = http.POST(payload);

        if (httpResponseCode > 0) {
            Serial.print("Server response: ");
            Serial.println(httpResponseCode);
            Serial.println(http.getString());
        } else {
            Serial.println("Failed to send data to server.");
        }
        http.end();
    } else {
        Serial.println("Wi-Fi not connected!");
    }
}

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi!");
    Serial.println(WiFi.localIP());

    // Initialize LoRa
    LoRa.setPins(cs_pin, reset_pin, irq_pin);
    if (!LoRa.begin(433E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    Serial.println("LoRa Initialized");

    // Initialize RFID
    initRFID();

    pinMode(LED, OUTPUT);
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String packetData = "";

        while (LoRa.available()) {
            packetData += (char)LoRa.read();
        }

        Serial.println("Received LoRa packet: " + packetData);

        // Send the LoRa packet data to the server
        sendToServer(packetData);
    }

    readRFID();
}