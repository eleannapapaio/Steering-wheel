#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);                               // CE, CSN
const byte ADDRESSES[2][5] = {{0, 0, 0, 0, 1}, {0, 0, 0, 0, 2}};// Setting the two addresses. One for transmitting and one for receiving

float roll;
int timoniPin = A0;

void setup()
{

      Serial.begin(9600);

      if(radio.begin()){
        Serial.println("Radio begin success!");// Starting the radio communication
      }else{
        Serial.println("Radio begin error!");
        while(1);
      }
      radio.openWritingPipe(ADDRESSES[0]);    // Setting the address at which we will send the data
      radio.openReadingPipe(1, ADDRESSES[1]); // Setting the address at which we will receive the data
      radio.setPALevel(RF24_PA_MAX);          // You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
}
void loop()
{
      int timoniValue = analogRead(timoniPin);
      int timoniInput = map(timoniValue, 0, 890, -130, 130);

      radio.stopListening();                          // This sets the module as transmitter
      if(radio.write(&timoniValue, sizeof(timoniValue))){
        // Sending the data
        Serial.println("Data Sent");
      }else{
        Serial.println("Data Sent error!");
      }
      radio.startListening();                         // This sets the module as receiver
      if (radio.available())                          // Looking for incoming data
      {
            radio.read(&roll, sizeof(roll));
            Serial.print("Received roll value: ");
            Serial.println(roll);
      }
      Serial.print("Analog value: ");
      Serial.println(timoniValue);
      Serial.print("Mapped value: ");
      Serial.println(timoniInput);
      delay(1000);
}
