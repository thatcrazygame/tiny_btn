
#include "Manchester.h"
#include <Speck.h>

/*

  Manchester Receiver example
  
  In this example receiver will receive array of 10 bytes per transmittion

  try different speeds using this constants, your maximum possible speed will 
  depend on various factors like transmitter type, distance, microcontroller speed, ...

  MAN_300 0
  MAN_600 1
  MAN_1200 2
  MAN_2400 3
  MAN_4800 4
  MAN_9600 5
  MAN_19200 6
  MAN_38400 7

*/

#define RX_PIN 7
#define LED_PIN 13

uint8_t moo = 1;
#define BUFFER_SIZE 22
uint8_t buffer[BUFFER_SIZE];

uint8_t key[16] = {0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00};
size_t keySize = 16;

uint8_t encrypted[16];
uint8_t message[16];

Speck speck;

void setup() 
{
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, moo);
  Serial.begin(19200);
  
  man.setupReceive(RX_PIN, MAN_600);
  man.beginReceiveArray(BUFFER_SIZE, buffer);

  speck.setKey(key, keySize);
}

void loop() 
{
  if (man.receiveComplete()) 
  {
    uint8_t receivedSize = 0;

    //do something with the data in 'buffer' here before you start receiving to the same buffer again
    receivedSize = buffer[0];

    for(uint8_t i=1; i<receivedSize; i++) {
      encrypted[i-1] = buffer[i];
    }

    speck.decryptBlock(message, encrypted);

    uint8_t sender = message[0];
    uint8_t action = message[1];
    uint8_t counter[4] = {message[12], message[13], message[14], message[15]};

//    for(uint8_t i=0; i<receivedSize-1; i++) {
//      if (i > 11) {
//        counter[i-12] = message[i];
//      } else {
//        Serial.write(static_cast<char>(message[i]));
//      }
//    }

    if (action == 'a' || action == 'b' || action == 'c') {
      Serial.print("Sent by: ");
      Serial.write(sender);

      if (action == 'a') Serial.print(" | Action: Click       ");
      if (action == 'b') Serial.print(" | Action: Double Click");
      if (action == 'c') Serial.print(" | Action: Hold        ");
      
      Serial.print(" | Count: ");
      Serial.print(*((uint32_t*) counter));
      
      Serial.println();
    } else {
      Serial.println("Bad message");
    }


    man.beginReceiveArray(BUFFER_SIZE, buffer);
    moo = ++moo % 2;
    digitalWrite(LED_PIN, moo);
  }
}
