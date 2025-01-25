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

char* jacket_id_ = "D7 C1 80 35";
int water_raw_val = 0; // value for storing water level
uint8_t sos_mode_triggered = 0; // gets set to 1 if panic button is pressed OR water threshold is set
uint8_t sos_mode_by_water = 0;
uint8_t sos_mode_by_button = 0;

const long BLINK_INTERVAL = 2000;   // interval at which to blink LED (milliseconds)
int ledState = LOW;   // ledState used to set the LED
unsigned long previousLEDMillis = 0;   // will store last time LED was updated
int button_pressed = 0;

// button press variables
#define DEBOUNCE_DELAY 50
int lastSteadyState = LOW; // previous steady state from the input pin
int lastFlickerableState = LOW; // previous flickerable state from the input pin
int currentState; // the current reading from the input pin
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long lastSendTime = 0;
volatile int buttonPressed = 0;

//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;
unsigned long last_button_time = 0;

char lora_msg[128];
char safe_msg[128];
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
void readGPS();
int readWaterLevel();
int readPanicButton();
void activateLEDs();
void sendLORAMsg(char* );
void initHW();
void panicButtonISR();
void sendSOS();
void sendSafeMsg();

/**
 * Functions implementation
 */

/**
initialize buzzer
**/
void buzz() {
    Serial.println("Buzzong");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
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

void IRAM_ATTR panicButtonISR() {
   // buzz();
    button_time = millis();
    if (button_time - last_button_time > 200) {
        button_pressed = 1;
        last_button_time = button_time;
    }

}

/**
 * @brief Init panic button
 */
void initPanicButton() {
    pinMode(PANIC_BUTTON, INPUT);
    attachInterrupt(PANIC_BUTTON, panicButtonISR, FALLING);
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
 * @brief Read GPS 
 */
void readGPS() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());

        if (gps.location.isUpdated()){ 
            // Get location
            //debug("Location: ");
            if(gps.location.isValid()) {
                latitude = gps.location.lat();
                //Serial.print(gps.location.lat(), 2);
                //debug(F(","));

                longitude = gps.location.lng();
                //Serial.print(gps.location.lng(), 2);

            } else {
                //debug(F("INVALID"));
            }

            // Get time and date
            //debug(F("Date/time: "));
            if(gps.date.isValid()) {
                month = gps.date.month();
                //debug(month);
                //debug(F("/"));

                day = gps.date.day();
                //debug(day);
                //debug(F("/"));

                year = gps.date.year();
                //debug(year);
            } else {
                //debug(F("INVALID"));
            }

            // time 
            //debug(F(" "));
            if (gps.time.isValid()) {

                hr = gps.time.hour();
                if (hr < 10) debug(F("0"));
                //debug(hr);

                mint = gps.time.minute();
                if (mint < 10) debug(F("0"));
                //debug(mint);

                sec = gps.time.second();
                if (sec < 10) debug(F("0"));
                //debug(sec);

            } else {
                //debug(F("INVALID"));
            }

            //debugln();
        }
    }

}

/**
 * @brief Read water level
 * 
 */
int readWaterLevel() {
    digitalWrite(WATER_LEVEL_PWR, HIGH); // turn the sensor ON
    delay(10);
    water_raw_val = analogRead(WATER_LEVEL_ADC_PIN);
    digitalWrite(WATER_LEVEL_ADC_PIN, LOW);
    return water_raw_val;
}

void initBuzzer() {
    pinMode(BUZZER_PIN, OUTPUT);
}

/**
 * @brief Initialize the hardware
 * 
 */
void initHW() {
    Serial.begin(BAUDRATE);
    initBuzzer();
    initLORA();
    initGPS();
    initWaterLevelSensor();
    initLEDs();
    initPanicButton();

}


void sendSOS() {

    Serial.println(lora_msg);
    LoRa.beginPacket();
    LoRa.print(lora_msg);
    LoRa.endPacket();
    delay(LORA_DELAY);
}

void sendSafeMsg() {

    Serial.println(safe_msg);
    LoRa.beginPacket();
    LoRa.print(safe_msg);
    LoRa.endPacket();

    delay(LORA_DELAY);
}

void setup() {
    debugln("Initializing hardware");
    initHW();
    debugln();

}

void loop() {

    // get GPS
    readGPS();

    // compose SAFE LORA message
    sprintf(safe_msg, "%s,OK,%.2f,%.2f,%d,%d,%d,%d,%d,%d\n",
            jacket_id_,
            latitude,
            longitude,
            hr+3,
            mint,
            sec,
            day,
            month,
            year
    );

    // compose SOS LORA message
    sprintf(lora_msg, "%s,SOS,%.2f,%.2f,%d,%d,%d,%d,%d,%d\n",
            jacket_id_,
            latitude,
            longitude,
            hr+3,
            mint,
            sec,
            day,
            month,
            year
            );

    // ================ CHECK FOR SOS =================
    // panic button pressed
    if(button_pressed) {
        sos_mode_by_button = 1;
        button_pressed = 0;
        buzz();
    }

    // read and check water level
    int d = readWaterLevel();
    // check for water level
    if(d > WATER_LEVEL_THRESHOLD) {
        sos_mode_by_water = 1;
        buzz();
        digitalWrite(LED1, HIGH);
    } else {
        // out of water, out of danger
        sos_mode_by_water = 0;
    }

    // here, SOS mode is triggered if any of the two methods detect danger
    // set SOS mode if the button is pressed OR the water level threshold is recognised
    if(sos_mode_by_button || sos_mode_by_water) {
        sos_mode_triggered = 1;
    } else {
        sos_mode_triggered = 0;
    }
    
    // here we check to see if SOS mode has been activated by any of the two methods
    // if any of them has occured, the person is in danger
    if(sos_mode_triggered) {
        sendSOS();
    } else {
        sendSafeMsg(); 
    }

    // LOWEST PRIORITY 
    // non-blocking LED blink
    unsigned long currentMillis = millis();
    if (currentMillis - previousLEDMillis >= BLINK_INTERVAL) {
        // if the LED is off turn it on and vice-versa:
        ledState = (ledState == LOW) ? HIGH : LOW;
        // set the LED with the ledState of the variable:
        digitalWrite(LED1, ledState);
        // save the last time you blinked the LED
        previousLEDMillis = currentMillis;
    }

}