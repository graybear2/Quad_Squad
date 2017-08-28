#include <Servo.h>
#include "Gyro.h"
#define TR 4
#define TL 5
#define BL 6
#define BR 7

String readString;
char control;
Gyro imu;
Servo topRight; //CLOCKWISE
Servo topLeft; 
Servo bottomLeft; //CLOCKWISE
Servo bottomRight;


//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;

//Setup routine
void setup(){
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change
  topRight.attach(TR); 
  topLeft.attach(TL);
  bottomLeft.attach(BL);
  bottomRight.attach(BR);
  topRight.write(0); // writes to ports
  topLeft.write(0);
  bottomLeft.write(0);
  bottomRight.write(0);
  imu.initSensors();
}

//Main program loop
void loop(){
  driveMotors();
}

//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT0_vect){
  //Channel 1=========================================
  if(last_channel_1 == 0 && PINB & B00000001 ){         //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){  //Input 8 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    receiver_input_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }
  //Channel 2=========================================
  if(last_channel_2 == 0 && PINB & B00000010 ){         //Input 9 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010)){  //Input 9 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }
  //Channel 3=========================================
  if(last_channel_3 == 0 && PINB & B00000100 ){         //Input 10 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100)){  //Input 10 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }
  //Channel 4=========================================
  if(last_channel_4 == 0 && PINB & B00001000 ){         //Input 11 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){  //Input 11 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }
}

void driveMotors(){
  int32_t throttle          = receiver_input_channel_1;
  int32_t rightRollPositive = receiver_input_channel_2;
  int32_t upPositive        = receiver_input_channel_3;
  int32_t rightYawPositive  = receiver_input_channel_4;
  int32_t leftRollPositive  = rightRollPositive;
  int32_t downPositive      = upPositive;
  int32_t leftYawPositive   = rightYawPositive;
  int32_t trMotor;
  int32_t tlMotor;
  int32_t blMotor;
  int32_t brMotor;
  
  if(rightRollPositive < 1500) {rightRollPositive = 1500;}
  if(upPositive        < 1500) {upPositive        = 1500;}
  if(rightYawPositive  < 1500) {rightYawPositive  = 1500;}
  if(leftRollPositive  > 1500) {leftRollPositive  = 1500;}
  if(downPositive      > 1500) {downPositive      = 1500;}
  if(leftYawPositive   > 1500) {leftYawPositive   = 1500;}

#define MIN 750 //assigns MIN as 750 (done this way bc it uses less memory than "int MIN = 750")
  throttle          = map(throttle,          1000, 2000, 0,    1000);
  rightRollPositive = map(rightRollPositive, 1500, 2000, MIN,  1000);
  leftRollPositive  = map(leftRollPositive,  1000, 1500, 1000, MIN );
  upPositive        = map(upPositive,        1500, 2000, MIN,  1000);
  downPositive      = map(downPositive,      1000, 1500, 1000, MIN );
  rightYawPositive  = map(rightYawPositive,  1500, 2000, MIN,  1000);
  leftYawPositive   = map(leftYawPositive,   1000, 1500, 1000, MIN );
  //AT THIS POINT: every input has been scaled to range from MIN to 1000
  //EXCEPT for throttle which ranges from 0 to 1000

  //Here is where you would give motor input ranging from 1000-2000
//  topRight.writeMicroseconds(<some calculated number>);
//  topLeft.writeMicroseconds(<some calculated number>);
//  bottomLeft.writeMicroseconds(<some calculated number>);
//  bottomRight.writeMicroseconds(<some calculated number>);
}

