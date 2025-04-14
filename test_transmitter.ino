#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);
const byte addresses[][6] = {"00001", "00002"};

void setup() {
  Serial.begin(115200);
  radio.begin();

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  radio.setPALevel(RF24_PA_MIN);
      radio.stopListening();

}

void loop() {

  float data=10.2;

//    uint32_t now = millis();
    while (!radio.write(&data, sizeof(data))){
      Serial.println("aaaa");
    } 
}
