

#include <Arduino.h>
#define DISABLE_CODE_FOR_RECEIVER // Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not used.
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>


int base1 = 6; // base of the first transistor 
int base2 = 7; // base of the second transistor 
int base3 = 8; // base of the third transistor 
int base4 = 9; // base of the fourth transistor 

void setup() {

    pinMode(base1,OUTPUT);
    pinMode(base2,OUTPUT);
    pinMode(base3,OUTPUT);
    pinMode(base4,OUTPUT);
    
    pinMode(IR_SEND_PIN, OUTPUT); // pin used to send IR signal, defined in PinDefinitionsAndMore, the pin used it the pin number 3 but it can be changed in the H file, any digital pin can be used.
    IrSender.begin(DISABLE_LED_FEEDBACK); // Start with IR_SEND_PIN as send pin and disable feedback LED at default feedback LED pin

}

uint8_t sCommand;
uint8_t sRepeats = 4; // number of time to repeat the signal 

void loop() {
    
    digitalWrite(base1,1);  // activate first base to send signal to LED1 , all other leds are deactivated
    digitalWrite(base2,0);  // keep all the other transistor deactivated
    digitalWrite(base3,0);
    digitalWrite(base4,0);
    sCommand = 0x34; // first command to send
    IrSender.sendNEC(0x04, sCommand, sRepeats); 
    // send IR data with NEC protocol, OX04 is the adress, scommand is the command ( max of 8 bits) , srepeats is the number of time that the signal will repeat itsel
    //aka the number of time it will be sent
    digitalWrite(base1,0);
    
    delay(10); // delay between each signal is neccesary to ensure that no signal is not modified by another one. a minimum of 5 ms is needed
    sCommand = 0x77;
    digitalWrite(base2,1);
    IrSender.sendNEC(0x07, sCommand, sRepeats);
    digitalWrite(base2,0);
    delay(10);  
  
    sCommand = 0x01;
    digitalWrite(base3,1);
    IrSender.sendNEC(0x07, sCommand, sRepeats);
    digitalWrite(base3,0);
    delay(10); 

    sCommand = 0x90;
    digitalWrite(base4,1);
    IrSender.sendNEC(0x07, sCommand, sRepeats);
    digitalWrite(base4,0);
    delay(10); 
}
