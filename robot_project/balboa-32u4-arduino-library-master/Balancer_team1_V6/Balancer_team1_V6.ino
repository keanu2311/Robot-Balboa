
#define DECODE_NEC   
#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <IRremote.hpp>
#include <HCSR04.h>

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;


bool IRsignal1 = false; // initialize signal to false AKA, no signal has been detected
bool IRsignal2 = false;
bool IRsignal3 = false;
bool IRsignal4 = false;
int16_t leftSpeed;
int16_t rightSpeed;

bool forward;  // conditionnal variable used for obstacle detection
bool in_function = false; // used to determine if the ultrasonic function has been passed in
double dist = 0;  // distance = 0 for ultra sonic function

unsigned long count_millis = 0;         // initialize counter to 0, used in ultra sonic function
unsigned long delay_forward = 1000;     // max allowed time before code must to re read values if signal has been detected
unsigned long delay_rotate = 200;       // max allowed time before code must to re read values if signal has  not been detected
bool detected_IR = false;

#define TRIG_PIN1 21    // pins for ultraSonic sensors
#define ECHO_PIN1 22
UltraSonicDistanceSensor distanceSensor1(TRIG_PIN1, ECHO_PIN1); //initialize sensor


void setup()
{
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // begin receiving  IR data, with ked indicator to know when a signal has been received
   ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
}


void IR_detectionV2()
// function used to decode the incomming IR signal
{
  IRsignal1 = false; 
  IRsignal2 = false; 
  IRsignal3 = false; 
  IRsignal4 = false;
  detected_IR = false;
 if (IrReceiver.decode())  // if signal has been decoded store wich signal came in a variable
  {
    
    IrReceiver.resume(); // Enable receiving of the next value
    switch (IrReceiver.decodedIRData.command)
     {
        case 0x34:
            IRsignal1 = true;
            detected_IR = true;
            break;
        case 0x77:
            IRsignal2 = true;
            detected_IR = true;
            break;
        case 0x01:
            IRsignal3 = true;
            detected_IR = true;
            break;
        case 0x90:
            IRsignal4 = true;
            detected_IR = true;
            break;
        default:
            break;
      }
  }
  
}

void UltraSound_sensor()
// function to calculate the distance to an object
{   
                                    
    dist=distanceSensor1.measureDistanceCm();   // distance is stored in the variable dist
    
    
    if ((dist>15)) // if distance is supperior to 15 cm then robot may continue to go forward
    {              
      forward = true;
      
    }
    else 
    {       // stop robot if < 15 cm
      forward = false;
    }
    
    count_millis=millis();  // start counter to determine how long we stayed in this function
    in_function = true; // to know if we passed thourgh the function
}


void loop()
{
 
  balanceUpdate();

 
  if (in_function == false) // first time in the code will start checking for obstacles as in function is declared as false
    {
     UltraSound_sensor();
    } 

  if ((IRsignal1 || IRsignal2 || IRsignal3 || IRsignal4) == false) // first time in the code will start checking for IR data as IR signals have been set to false at the start
    {
      IR_detectionV2();
    }
    
  
    if (detected_IR == true)                                       
      {
        leftSpeed = -10;     // speed of both motors are put as the same to go forward                           
        rightSpeed = -10;
        balanceDrive(leftSpeed, rightSpeed);  // pass in leftSpeed and rightSpeed in the function to go forward   
        if ((millis()-count_millis) > delay_forward) // if the elasped time since the start of the code and the time we stayed in the ultrasonics function is greater then 1000ms then we reread a value
          { 
            in_function = false; // reset to false to check for obstalces once the loop function restarts      
            IR_detectionV2(); 
          }
        if (forward == false) // if we detected an obstacle, the robot must stop. 
        {
          leftSpeed = 0;                               
          rightSpeed = 0;
          balanceDrive(leftSpeed, rightSpeed);
     
        }
      }
     
    if (detected_IR == 0) // if no IR signal has been detected, we rotate the robot until it detects it again, this allows to course correct to reach the base. 
      {
        leftSpeed = -5;
        rightSpeed = 5;
        balanceDrive(leftSpeed, rightSpeed);
        if ((millis()-count_millis)>delay_rotate)
          {
            in_function = false;
            IR_detectionV2();
          }
      }
     
  int32_t fallingAngleOffset = angleRate * ANGLE_RATE_RATIO - angle;
  if (fallingAngleOffset > 0)
  {
    ledYellow(1);
    ledGreen(0);
  }
  else
  {
    ledYellow(0);
    ledGreen(1);
  }
  
}
