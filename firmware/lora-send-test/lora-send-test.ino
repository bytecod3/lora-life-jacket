
#include <LoRa.h>
#include <SPI.h>

const int cs_pin = 4; 
const int reset_pin = 2;
const int irq_pin = 22; // DIO0 on HOPE RF LORA MODULE - pin must have hardware interrupt

// message counter
byte message_count = 0;
String received_msg = "";
char rcvd_msg_buff[100];
int counter = 0;

void setup() {
   Serial.begin(115200);

   LoRa.setPins(cs_pin, reset_pin, irq_pin);
   Serial.println("LORA Send test");

   if(!LoRa.begin(433E6)) {
    Serial.println("Lora failed to start");
   } else {
    Serial.println("Lora started OK!");
   }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Sending packet: ");
  Serial.println(counter);
  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();
  counter++;
  delay(5000);
}
