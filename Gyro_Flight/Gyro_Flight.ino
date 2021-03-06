#include <PID_v1.h>
#include <Servo.h>
#include "Gyro.h"
#include <math.h>
#define TR 4
#define TL 5
#define BL 6
#define BR 7
#define DEBUG 1

Gyro imu;
Servo topRight; //CLOCKWISE
Servo topLeft; 
Servo bottomLeft; //CLOCKWISE
Servo bottomRight;

// Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4;
double nowPitch, setPitch, goalPitch;
#define KpPitch 50
#define KiPitch 5
#define KdPitch 0
double nowRoll, setRoll, goalRoll;
#define KpRoll 50
#define KiRoll 5
#define KdRoll 0
double nowYaw, setYaw, goalYaw;
#define KpYaw 50
#define KiYaw 5
#define KdYaw 0
float xAccelRaw, yAccelRaw, zAccelRaw, accelMag;
float pitchGyroRaw, rollGyroRaw, yawGyroRaw;

// Create PID controllers
PID pidPitch(&nowPitch, &setPitch, &goalPitch, KpPitch, KiPitch, KdPitch, DIRECT);
PID pidRoll(&nowRoll, &setRoll, &goalRoll, KpRoll, KiRoll, KdRoll, DIRECT);
PID pidYaw(&nowYaw, &setYaw, &goalYaw, KpYaw, KiYaw, KdYaw, DIRECT);

// Setup routine
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
  if(DEBUG)
    Serial.begin(115200);
}

//Main program loop
void loop(){

   // get raw linear acceleration data from IMU
   xAccelRaw = imu.getAccelX();
   yAccelRaw = imu.getAccelY();
   zAccelRaw = imu.getAccelZ();

   // calculate magnitude of acceleration
   accelMag = sqrt(sq(xAccelRaw) + sq(yAccelRaw) + sq(zAccelRaw));

   // ignore outliers or weird accelerometer spikes
   if(accelMag > .5 && accelMag < 4) {
    // calculate pitch angle based on accelerometer
    float pitchAccel = atan2(-yAccelRaw, zAccelRaw) * 180.0 / M_PI;
    
    // calculate roll angle based on accelerometer
    float rollAccel = atan2(xAccelRaw, sqrt(sq(yAccelRaw) + sq(zAccelRaw))) * 180.0 / M_PI;

    // use complementary filter to improve angle estimation on pitch and roll
    nowPitch = compFilter(pitchGyroRaw, pitchAccel);
    nowRoll = compFilter(rollGyroRaw, rollAccel);    
   }
   else {
    // if acceleromter data is weird, ignore
    nowPitch = rollGyroRaw;
    nowRoll = pitchGyroRaw;
   }
   // yaw is not affected by complementary filter
   nowYaw = imu.getYawOrientation();


  if(DEBUG){
     // for complementary filter testing only
     Serial.print(F("Pitch: "));
     Serial.print(nowPitch);
     Serial.print(F("/tRoll: "));
     Serial.print(nowRoll);
     Serial.print(F("/tYaw: "));
     Serial.print(nowYaw);
     Serial.println("");
  }
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

  #define MIN 500
  throttle          = map(throttle,          1000, 2000, 0,    1000);
  rightRollPositive = map(rightRollPositive, 1000, 2000, MIN,  1000);
  leftRollPositive  = map(leftRollPositive,  1000, 2000, 1000, MIN );
  upPositive        = map(upPositive,        1000, 2000, MIN,  1000);
  downPositive      = map(downPositive,      1000, 2000, 1000, MIN );
  rightYawPositive  = map(rightYawPositive,  1000, 2000, MIN,  1000);
  leftYawPositive   = map(leftYawPositive,   1000, 2000, 1000, MIN );
  //AT THIS POINT: every input has been scaled to range from MIN to 1000
  //EXCEPT for throttle which ranges from 0 to 1000

  //Here is where you would give motor input ranging from 1000-2000
//  topRight.writeMicroseconds(<some calculated number>);
//  topLeft.writeMicroseconds(<some calculated number>);
//  bottomLeft.writeMicroseconds(<some calculated number>);
//  bottomRight.writeMicroseconds(<some calculated number>);
}

double compFilter(double gyroAngle, double accelAngle) {
  // based on testing the .98 and .02 can be changed to improve filtering
  double angleFiltered = (gyroAngle * .98) + (accelAngle * .02);
  return angleFiltered;  
}

