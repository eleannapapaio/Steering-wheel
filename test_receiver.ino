#include <SPI.h>            
#include <nRF24L01.h>       
#include <RF24.h>    
#include <stdint.h>

RF24 radio(9,10);            
const byte addresses[][6] = {"00001", "00002"};


void setup() {
Serial.begin(115200);           
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
      radio.startListening();
}

void loop() {
  Serial.print("aaa\n");
    if(radio.available()){
      float data;
      radio.read(&data, sizeof(data));
      Serial.print("Response=");
      Serial.println(data);
    }
}
