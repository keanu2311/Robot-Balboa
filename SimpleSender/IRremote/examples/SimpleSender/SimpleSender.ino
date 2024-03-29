/*
 * SimpleSender.cpp
 *
 *  Demonstrates sending IR codes in standard format with address and command
 *  An extended example for sending can be found as SendDemo.
 *
 *  Copyright (C) 2020-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 *  MIT License
 */
#include <Arduino.h>

#define DISABLE_CODE_FOR_RECEIVER // Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not used.
//#define SEND_PWM_BY_TIMER         // Disable carrier PWM generation in software and use (restricted) hardware PWM.
//#define USE_NO_SEND_PWM           // Use no carrier PWM, just simulate an active low receiver signal. Overrides SEND_PWM_BY_TIMER definition

#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>
int base=8;
int base2=9;
void setup() {

    pinMode(base,OUTPUT);
    pinMode(base2,OUTPUT);
    pinMode(IR_SEND_PIN, OUTPUT);
    
    // Just to know which program is running on my Arduino
    //Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    /*
     * The IR library setup. That's all!
     */
    //IrSender.begin(); // Start with IR_SEND_PIN as send pin and if NO_LED_FEEDBACK_CODE is NOT defined, enable feedback LED at default feedback LED pin
    IrSender.begin(DISABLE_LED_FEEDBACK); // Start with IR_SEND_PIN as send pin and disable feedback LED at default feedback LED pin

    //Serial.print(F("Send IR signals at pin "));
    //Serial.println(IR_SEND_PIN);
}

/*
 * Set up the data to be sent.
 * For most protocols, the data is build up with a constant 8 (or 16 byte) address
 * and a variable 8 bit command.
 * There are exceptions like Sony and Denon, which have 5 bit address.
 */
uint8_t sCommand = 0x34;
uint8_t sRepeats = 0;

void loop() {
    /*
     * Print current send values
     */
    //Serial.println();
    //Serial.print(F("Send now: address=0x00, command=0x"));
    //Serial.print(sCommand, HEX);
    //Serial.print(F(", repeats="));
    //Serial.print(sRepeats);
    //Serial.println();

    //Serial.println(F("Send standard NEC with 8 bit address"));
    //Serial.flush();

    // Receiver output for the first loop must be: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    digitalWrite(base2,0);
    digitalWrite(base,1);
    sCommand = 0x34;
    IrSender.sendNEC(0x04, sCommand, sRepeats);
    //delay(2000);
    sCommand = 0x77;
    digitalWrite(base,0);
    digitalWrite(base2,1);
    IrSender.sendNEC(0x04, sCommand, sRepeats);
  

    /*
     * Increment send values
     */
    //sCommand += 0x11;
    sRepeats++;
    // clip repeats at 4
    if (sRepeats > 4) {
        sRepeats = 4;
    }

    delay(20);  // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
}
