


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
 */



#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <OneButton.h>
#include <Manchester.h>
#include <SpeckTiny.h>
#include <EEPROM.h>

//#define LED_PIN 3    // for testing
//#define SLEEP_LED 4  // for testing

#define TX_PIN 0

OneButton button(A1, true);
//uint8_t moo = 1;  // for testing

uint8_t data[18];
uint8_t datalength = 17;
uint8_t key[16] = {0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00};
uint8_t message[16] = {'a','a','a','a','a','a','a','a','a','a','a','a','a','a','a','a'};
uint8_t encrypted[16];

uint8_t count_mem_idx = 0;
uint32_t counter = 0;

bool sending = 0;

size_t keySize = 16;

SpeckTiny speckTiny;

void setup() {
  data[0] = datalength;
  data[datalength] = 0;
  
  speckTiny.setKey(key, keySize);
  man.setupTransmit(TX_PIN, MAN_600);

  button.setClickTicks(400);
  button.attachClick(singleclick);     
  button.attachDoubleClick(doubleclick);
  button.attachLongPressStop(longclick);

  count_mem_idx = EEPROM.read(0);
  if (count_mem_idx == 0) count_mem_idx = 1;
 
  EEPROM.get(count_mem_idx, counter);
  if (counter == 4294967295) counter = 0; // not sure why the first read after clearing the EEPROM was this value
  
  if (count_mem_idx != calc_mem_idx(counter)) {
    // TODO: ERROR mistmatch in index
  }
  
    
//  pinMode(LED_PIN, OUTPUT);    // for testing
//  pinMode(SLEEP_LED, OUTPUT);  // for testing
}

void loop() {
  delay(10);
  button.tick();

  if (button.getState() == 0){
    // Only sleep when returning to state 0 (waiting for click). 
    // First click will cause interrupt to break the sleep.
    sleep(); 
  }
  
} // loop


void send_encrypted_action(uint8_t sender, uint8_t action) {
  if ( ! sending ) {
    sending = true;
    
    message[0] = sender;
    message[1] = action;

    message[15] = (uint8_t)(counter>>24);
    message[14] = (uint8_t)(counter>>16);
    message[13] = (uint8_t)(counter>>8);
    message[12] = (uint8_t)(counter>>0);

    speckTiny.encryptBlock(encrypted, message);
  
    for (uint8_t i=1; i<datalength; i++) {
      data[i] = encrypted[i-1];  
    }
    
    man.transmitArray(datalength, data);
  
    counter++;

    count_mem_idx = calc_mem_idx(counter);
    EEPROM.update(0, count_mem_idx);
    EEPROM.put(count_mem_idx, counter);
    
//    moo = ++moo % 2; //for testing
//    digitalWrite(LED_PIN, moo); // for testing

    sending = false;
  }
}



void sleep() {
//    digitalWrite(SLEEP_LED, HIGH); // for testing

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
    
//    digitalWrite(SLEEP_LED, LOW);  // for testing
} // sleep

uint8_t calc_mem_idx(uint32_t counter) {
  /*
   * EEPROM limit of 100,000 write cycles
   * This will iterate the counter through the indexes 
   * so it doesn't write the same location too much.
   * Might be overdoing it, but better safe than sorry.
   */
  return (counter / 99000) + 1;
}

ISR(PCINT0_vect) {
  // don't need to do anything here
}

void singleclick(){
  send_encrypted_action('z', 'a');
}

void doubleclick() {
  send_encrypted_action('z', 'b');
}
 
void longclick(){
  send_encrypted_action('z', 'c');
}
