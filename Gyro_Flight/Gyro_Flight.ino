#include <PID_v1.h>
#include <Servo.h>
#include "Gyro.h"
#include <math.h>
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

// Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;
double nowPitch, setPitch, goalPitch, KpPitch, KiPitch, KdPitch;
double nowRoll, setRoll, goalRoll, KpRoll, KiRoll, KdRoll;
double nowYaw, setYaw, goalYaw, KpYaw, KiYaw, KdYaw; 
float xAccelRaw, yAccelRaw, zAccelRaw, accelMag;
float pitchGyroRaw, rollGyroRaw, yawGyroRaw;

// Create PID controllers
PID pidPitch(&nowPitch, &setPitch, &goalPitch, KpPitch, KiPitch, KdPitch, DIRECT);
PID pidRoll(&nowRoll, &setRoll, &goalRoll, KpRoll, KiRoll, KdRoll, DIRECT);
PID pidYaw(&nowYaw, &setYaw, &goalYaw, KpYaw, KiYaw, KdYaw, DIRECT);

// Setup routine
void setup(){
  // Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
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
  Serial.begin(9600);
}

// Main program loop
void loop(){
   xAccelRaw = imu.getAccelX();
   yAccelRaw = imu.getAccelY();
   zAccelRaw = imu.getAccelZ();
   
   accelMag = sqrt(sq(xAccelRaw) + sq(yAccelRaw) + sq(zAccelRaw));
   if(accelMag > .5 && accelMag < 4) {
    float pitchAccel = atan2(-yAccelRaw, zAccelRaw) * 180.0 / M_PI;
    float rollAccel = atan2(xAccelRaw, sqrt(sq(yAccelRaw) + sq(zAccelRaw))) * 180.0 / M_PI;
    nowPitch = compFilter(pitchGyroRaw, pitchAccel);
    nowRoll = compFilter(rollGyroRaw, rollAccel);    
   }
   else {
    nowPitch = rollGyroRaw;
    nowRoll = pitchGyroRaw;
   }
   nowYaw = imu.getYawOrientation();

   Serial.print(F("Pitch: "));
   Serial.print(nowPitch);
   Serial.print(F("/tRoll: "));
   Serial.print(nowRoll);
   Serial.print(F("/tYaw: "));
   Serial.print(nowYaw);
   Serial.println("");
  
  driveMotors();
}

// This routine is called every time input 8, 9, 10 or 11 changed state
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
//Subroutine for displaying the receiver signals
void print_signals(){
  Serial.print("Roll:");
  if(receiver_input_channel_2 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_2 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);
  
  Serial.print("  Pitch:");
  if(receiver_input_channel_3 - 1480 < 0)Serial.print("^^^");
  else if(receiver_input_channel_3 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);
  
  Serial.print("  Gas: ");
  if(receiver_input_channel_1 - 1480 < 0)Serial.print("vvv   ");
  else if(receiver_input_channel_1 - 1520 > 0)Serial.print("^^^   ");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_1);
  
  Serial.print("  Yaw:");
  if(receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
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
  
//  throttle -= 1000;
//  rightRollPositive /= 2;
//  rightRollPositive *= throttle;
//  rightRollPositive /= 1000;
//  leftRollPositive /= -2;
//  leftRollPositive += 1500;
//  leftRollPositive *= throttle;
//  leftRollPositive /= 1000;
//  upPositive /= 2;
//  upPositive *= throttle;
//  upPositive /= 1000;
//  downPositive /= -2;
//  downPositive += 1500;
//  downPositive *= throttle;
//  downPositive /= 1000; 
//  rightYawPositive /= 4;
//  rightYawPositive += 500;
//  leftYawPositive /= -4;
//  leftYawPositive += 1250;
//
//  trMotor = leftRollPositive*downPositive;  //CLOCKWISE
//  trMotor /= 1000;
//  trMotor *= rightYawPositive;
//  trMotor /= 1000;
//
//  tlMotor = rightRollPositive*downPositive;
//  tlMotor /= 1000;
//  tlMotor *=leftYawPositive;
//  tlMotor /= 1000;
//
//  blMotor = rightRollPositive*upPositive;
//  blMotor /= 1000;
//  blMotor *= rightYawPositive;
//  blMotor /= 1000;
//
//  brMotor = leftRollPositive*upPositive;
//  brMotor /= 1000;
//  brMotor *= leftYawPositive;
//  brMotor /= 1000;

//  Serial.print("TR: ");
//  Serial.print(trMotor);
//  Serial.print("   TL: ");
//  Serial.print(tlMotor);
//  Serial.print("   BL: ");
//  Serial.print(blMotor);
//  Serial.print("   BR: ");
//  Serial.println(brMotor);

  throttle -= 1000;
  rightRollPositive = map(rightRollPositive, 1000, 2000, 500, 1000);
  leftRollPositive  = map(leftRollPositive,  1000, 2000, 1000, 500);

  topRight.writeMicroseconds(trMotor+1000);
  topLeft.writeMicroseconds(tlMotor+1000);
  bottomLeft.writeMicroseconds(blMotor+1000);
  bottomRight.writeMicroseconds(brMotor+1000);
}

double compFilter(double gyroAngle, double accelAngle) {
  double angleFiltered = (gyroAngle * .98) + (accelAngle * .02);
  return angleFiltered;  
}

