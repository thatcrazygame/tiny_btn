
#include <Manchester.h>
#include <Speck.h>

#include "myKey.h"

/*
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

/*
 * key[] now comes from the myKey.h file in the same directory as the .ino
 * Because the Ardunio IDE doesn't support relative include paths
 * you need to have a copy of the key in both the rx and tx sketch folders
 * I've added myKey.h in both locations to .gitignore 
 * I'm not using the key below, but these two lines are all that needs to be in myKey.h
 * uint8_t key[16] = {0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00};
 * size_t keySize = 16;
 */


#define SENDER_IDX    0
#define ACTION_IDX    1
#define COUNTER_IDX   11
#define CHECKSUM_IDX  15

#define CLICK       0
#define DBL_CLICK   1
#define HOLD        2

#define RX_PIN 7
#define LED_PIN 13
#define MESSAGELENGTH 16

#define BUFFER_SIZE 22

uint8_t buffer[BUFFER_SIZE];
uint8_t encrypted[MESSAGELENGTH];
uint8_t message[MESSAGELENGTH];
uint8_t led_state = 1;

Speck speck;

void setup() 
{
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, led_state);
  Serial.begin(19200);
  
  man.setupReceive(RX_PIN, MAN_600);
  man.beginReceiveArray(BUFFER_SIZE, buffer);

  speck.setKey(key, keySize);
}

void loop() 
{
  if (man.receiveComplete()) 
  {
    // I cannot figure out why I can't make this work by just assuming the recieved size and using the buffer directly
    uint8_t receivedSize = 16;
    receivedSize = buffer[0];

    for(uint8_t i=1; i<receivedSize; i++) {
      encrypted[i-1] = buffer[i];
    }

    speck.decryptBlock(message, encrypted);

    uint8_t sender = message[SENDER_IDX];
    uint8_t action = message[ACTION_IDX];
    uint8_t counter[4] = {message[COUNTER_IDX]
                        , message[COUNTER_IDX + 1]
                        , message[COUNTER_IDX + 2]
                        , message[COUNTER_IDX + 3]};
    
    byte checksum = CRC8(message, CHECKSUM_IDX);

    if (checksum == message[CHECKSUM_IDX]) {
      Serial.print("{\"success\":true, ");
      Serial.print("\"sender\": ");
      Serial.print(sender);
      Serial.print(", \"action\":");
      
      if (action == CLICK) {
        Serial.print("\"click\"");
      } else if (action == DBL_CLICK) {
        Serial.print("\"dbl_click\"");
      } else if (action == HOLD) {
        Serial.print("\"hold\"");
      } else {
        Serial.print("\"undefined\"");
      }
      
      Serial.print(", \"count\": ");
      Serial.print(*((uint32_t*) counter));
      Serial.print("}");
      
      Serial.println();
    } else {
      Serial.println("{\"success\":false}");
    }


    man.beginReceiveArray(BUFFER_SIZE, buffer);
    led_state = ++led_state % 2;
    digitalWrite(LED_PIN, led_state);
  }
}


byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
