/*
 * REFERENCES:
 * 
 * Sleep function
 * https://bigdanzblog.wordpress.com/2014/08/10/attiny85-wake-from-sleep-on-pin-state-change-code-example/
 * 
 * OneButton library
 * https://github.com/mathertel/OneButton
 * modified with simple getState() function
 * int OneButton::getState(void) {
 *    return _state;
 * }
 * 
 * Manchester library
 * https://github.com/mchr3k/arduino-libs-manchester
 * 
 * Speck and other cryptography libs
 * https://github.com/rweather/arduinolibs
 * 
 * CRC8 function
 * https://www.leonardomiliani.com/en/2013/un-semplice-crc8-per-arduino/
 * 
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

#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <OneButton.h>
#include <Manchester.h>
#include <SpeckTiny.h>
#include <EEPROM.h>

#include "myKey.h"

#define TX_PIN 0
#define DATALENGTH 17
#define MESSAGELENGTH 16

#define SENDER_IDX    0
#define ACTION_IDX    1
#define COUNTER_IDX   11
#define CHECKSUM_IDX  15

#define SENDER 3 // change for each button

#define CLICK       0
#define DBL_CLICK   1
#define HOLD        2

OneButton button(A1, true);

uint8_t data[DATALENGTH];
uint8_t message[MESSAGELENGTH];
uint8_t encrypted[MESSAGELENGTH];

uint8_t count_mem_idx = 0;
uint32_t counter = 0;

bool sending = 0;

SpeckTiny speckTiny;

void setup() {
  data[0] = DATALENGTH;
    
  speckTiny.setKey(key, keySize);
  man.setupTransmit(TX_PIN, MAN_600);

  button.setClickTicks(400);
  button.attachClick(singleclick);     
  button.attachDoubleClick(doubleclick);
  button.attachLongPressStop(longclick);

  count_mem_idx = EEPROM.read(0);
  if (count_mem_idx == 0) count_mem_idx = 1;
  EEPROM.get(count_mem_idx, counter);
  if (counter + 1 == 0) counter = 0; // first read after uploading code is at uint32 max for some reason
  if (count_mem_idx != calc_mem_idx(counter)) {
    // TODO: ERROR mistmatch in index
  }
  
    
//  pinMode(LED_PIN, OUTPUT);    // for testing
//  pinMode(SLEEP_LED, OUTPUT);  // for testing
} // setup

void loop() {
  delay(10);
  button.tick();

  if (button.getState() == 0){
    // Only sleep when returning to state 0 (waiting for click). 
    // First click will cause interrupt to break the sleep.
    sleep(); 
  }
  
} // loop

void singleclick(){
  send_encrypted_action(SENDER, CLICK);
}

void doubleclick() {
  send_encrypted_action(SENDER, DBL_CLICK);
}

void longclick(){
  send_encrypted_action(SENDER, HOLD);
}

void send_encrypted_action(uint8_t sender, uint8_t action) {
  if ( ! sending ) {
    sending = true;
    
    message[SENDER_IDX] = sender;
    message[ACTION_IDX] = action;

    message[COUNTER_IDX + 3] = (uint8_t)(counter>>24);
    message[COUNTER_IDX + 2] = (uint8_t)(counter>>16);
    message[COUNTER_IDX + 1] = (uint8_t)(counter>>8);
    message[COUNTER_IDX    ] = (uint8_t)(counter>>0);

    byte checksum = CRC8(message, CHECKSUM_IDX);
    message[CHECKSUM_IDX] = checksum;

    speckTiny.encryptBlock(encrypted, message);
  
    for (uint8_t i=1; i < DATALENGTH; i++) {
      data[i] = encrypted[i-1];  
    }
    
    man.transmitArray(DATALENGTH, data);
  
    counter++;
    count_mem_idx = calc_mem_idx(counter);
    EEPROM.update(0, count_mem_idx);
    EEPROM.put(count_mem_idx, counter);
    
//    moo = ++moo % 2; //for testing
//    digitalWrite(LED_PIN, moo); // for testing

    sending = false;
  }
} // send_encrypted_action

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
} // CRC8

void sleep() {
    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT2);                   // Use PB2/A1 as interrupt pin
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on

    sei();                                  // Enable interrupts
} // sleep

uint8_t calc_mem_idx(uint32_t counter) {
  /*
   * EEPROM limit of 100,000 write cycles
   * This will iterate the counter through the indexes 
   * so it doesn't write the same location too much.
   * Might be overdoing it, but better safe than sorry.
   */
  return (counter / 99000) + 1;
} //calc_mem_idx

ISR(PCINT0_vect) {
  // don't need to do anything here
}
