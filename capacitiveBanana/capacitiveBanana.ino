/*
  banana Keyboard

  For the itsybitsy atmega32u4 5V 16MHz

  Type 'banana ' when banana.

  The circuit:
  - banana attached to pin 12
  - person makes connection with banana

  created 12 Dec 2020
  by Sam Schwartz

  This banana is in the public domain.

  http://banana.com/ (I am not responsible for whatever this website is, I have no idea)
*/

#include <Keyboard.h>

//Mode 1 is for running, mode 0 is for testing
#define MODE 1
// Set this to the number output steadily by testing mode while holding solid contact with banana (will be between 0 and 17, inclusive)
#define THRESHOLD 3
// Minimum time (in ms) to wait before sending another input (if too low, there may be bounce)
#define DEBOUNCE 100

const int bananaPin = 12;
bool previousBananaState = false;   // for checking the state of banana
void setup() {
  #if MODE == 0
    Serial.begin(9600);
  #elif MODE == 1
    pinMode(LED_BUILTIN, OUTPUT);
    // initialize control over the keyboard:
    Keyboard.begin();
  #endif
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  bool bananaState;
  // determine capacitance of banana
  uint8_t touchy=readCapacitivePin(bananaPin);
  #if MODE == 0
    Serial.println(touchy);
  #elif MODE == 1
    // if capacitance is high enough, banana has been poked
    if(touchy>=THRESHOLD){
      bananaState=true;
    }
    else{
      bananaState=false;
    }
    // if the banana state has changed,
    if ((bananaState != previousBananaState)
        // and it's currently pressed:
        && (bananaState == true)) {
      digitalWrite(LED_BUILTIN, HIGH);
      // banana
      Keyboard.print("banana ");
      delay(DEBOUNCE);
    }
    // save the current banana state for comparison next time:
    previousBananaState = bananaState;
  #endif
}

// readCapacitivePin
//  Input: Arduino pin number
//  Output: A number, from 0 to 17 expressing
//  how much capacitance is on the pin
//  When you touch the pin, or whatever you have
//  attached to it, the number will get higher

uint8_t readCapacitivePin(int pinToMeasure) {

  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;

  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));

  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);

  uint8_t SREG_old = SREG; //back up the AVR Status Register

  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();

  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
  if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // End of timing-critical section; turn interrupts back on if they were on before, or leave them off if they were off before
  SREG = SREG_old;

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}
