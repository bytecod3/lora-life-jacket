
#ifndef DEFS_H
#define DEFS_H

#define DEBUG 1

#if DEBUG
#define debug(x)   Serial.print(x);
#define debugln(x)  Serial.println(x);

#else 
#define debug(x) 
#define debugln(x)  
#endif // END DEBUG

// pin definitions
#define LORA_CS 4
#define LORA_RST    2
#define LORA_DIO    22
#define LORA_FREQUENCY 868E6
#define LORA_DELAY 1500

#define BAUDRATE 115200

// other pin defs
#define PANIC_LED 5


#endif