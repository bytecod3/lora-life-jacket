/*
 * This sketch receives data from LORA transmitter 
  * receives and displys message packet from lora
  * based upon DroneBot workshop code 
  * DroneBot Workshop 2023
  * https://dronebotworkshop.com
  * Author - Edwin Mwiti
  * July 20 2024
*/
#include <SPI.h>
#include <LoRa.h>

const int cs_pin = 4; 
const int reset_pin = 15;
const int irq_pin = 22; // DIO0 on HOPE RF LORA MODULE - pin must have hardware interrupt


void setup() {
  Serial.begin(115200);
 
  LoRa.setPins(cs_pin, reset_pin, irq_pin);
  Serial.println("LoRa Receiver");
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
  } else {
    Serial.println("LORA STARTED OK!");
  }
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
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());

  }

}
