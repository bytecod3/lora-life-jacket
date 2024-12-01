/*
 * This sketch receives data from LORA transmitter 
  * receives and displys message packet from lora
  * based upon DroneBot workshop code 
  * DroneBot Workshop 2023
  * https://dronebotworkshop.com
  * Author - Edwin Mwiti
  * July 20 2024
*/

#include <LoRa.h>
#include <SPI.h>

const int cs_pin = 4; 
const int reset_pin = 15;
const int irq_pin = 22; // DIO0 on HOPE RF LORA MODULE - pin must have hardware interrupt

// message counter
byte message_count = 0;

void setup() {
   Serial.begin(115200);

   LoRa.setPins(cs_pin, reset_pin, irq_pin);
   Serial.println("LORA receive test");

   if(!LoRa.begin(868E6)) {
    Serial.println("Lora failed to start");
   } else {
    Serial.println("Lora started OK!");
   }
}

void loop() {

  // parse packet
  int pkt_sz = LoRa.parsePacket();
  Serial.println(pkt_sz);
  if(pkt_sz) {
    // received packet
    Serial.print("[Received pkt] ");

    // Read packet
    while(LoRa.available()) {
      Serial.print((char) LoRa.read());
    }

    Serial.println();

    // // Print RSSI of the packet
    // Serial.print(" [RSSI]");
    // Serial.println(LoRa.packetRssi());
  }  

}
