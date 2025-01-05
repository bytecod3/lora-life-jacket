
#ifndef DEFS_H
#define DEFS_H

#define DEBUG 1

#if DEBUG
#define debug(x)   Serial.print(x);
#define debugln(x)  Serial.println(x);
#define debugf(x, y) Serial.printf(x, y)

#else 
#define debug(x) 
#define debugln(x)  
#endif // END DEBUG

// pin definitions
#define LORA_CS 4
#define LORA_RST    2
#define LORA_DIO    22
#define LORA_FREQUENCY 433E6
#define LORA_DELAY 10

#define BAUDRATE 115200
#define LED_BLINK   400

// other pin defs
#define LED1 26
#define PANIC_BUTTON 15

// water level pin
#define WATER_LEVEL_PWR 21
#define WATER_LEVEL_ADC_PIN 14
#define WATER_LEVEL_THRESHOLD 400

// GPS serial
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

#define BUZZER 25


#endif