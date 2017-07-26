#include <Servo.h>
#define TR 4
#define TL 5
#define BL 6
#define BR 7

String readString;
char control;
Servo topRight; //COUNTERCLOCKWISE
Servo topLeft;
Servo bottomLeft; //COUNTERCLOCKWISE
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
  Serial.begin(115200); 
  Serial.println("Program Started");
  control = 'n';
}

//Main program loop
void loop(){
  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (readString.length() > 0) {
    Serial.println(readString);  //so you can see the captured string
    int n = 0;
    if (readString == "a"){
      Serial.println("Attached motor");
      topRight.attach(TR); 
      topLeft.attach(TL);
      bottomLeft.attach(BL);
      bottomRight.attach(BR);
      Serial.print("writing Angle: "); 
      Serial.println(n); // number
      topRight.write(n); // writes to ports
      topLeft.write(n);
      bottomLeft.write(n);
      bottomRight.write(n);
    }
    else if(readString == "r"){
      Serial.println("Receiver Control");
      control = 'r';
    }
    else{
      n = readString.toInt();  //convert readString into a number
    }
    // auto select appropriate value, copied from someone elses code.
    if(n >= 500){
      Serial.print("writing Microseconds: "); // serial display
      Serial.println(n); // number
      control = 'n';
      topRight.writeMicroseconds(n);
      topLeft.writeMicroseconds(n);
      bottomLeft.writeMicroseconds(n);
      bottomRight.writeMicroseconds(n);
    }
  }
  if(control == 'r'){
    driveMotors();
  }
  else{
    print_signals();
    delay(500);
  }
  readString = "";
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
  
  throttle -= 1000;
  rightRollPositive /= 2;
  rightRollPositive *= throttle;
  rightRollPositive /= 1000;
  leftRollPositive /= -2;
  leftRollPositive += 1500;
  leftRollPositive *= throttle;
  leftRollPositive /= 1000;
  upPositive /= 2;
  upPositive *= throttle;
  upPositive /= 1000;
  downPositive /= -2;
  downPositive += 1500;
  downPositive *= throttle;
  downPositive /= 1000; 
  rightYawPositive /= 4;
  rightYawPositive += 500;
  leftYawPositive /= -4;
  leftYawPositive += 1250;

  trMotor = leftRollPositive*downPositive;  //CLOCKWISE
  trMotor /= 1000;
  trMotor *= leftYawPositive;
  trMotor /= 1000;

  tlMotor = rightRollPositive*downPositive;
  tlMotor /= 1000;
  tlMotor *=rightYawPositive;
  tlMotor /= 1000;

  blMotor = rightRollPositive*upPositive;
  blMotor /= 1000;
  blMotor *= leftYawPositive;
  blMotor /= 1000;

  brMotor = leftRollPositive*upPositive;
  brMotor /= 1000;
  brMotor *= rightYawPositive;
  brMotor /= 1000;

//  Serial.print("TR: ");
//  Serial.print(trMotor);
//  Serial.print("   TL: ");
//  Serial.print(tlMotor);
//  Serial.print("   BL: ");
//  Serial.print(blMotor);
//  Serial.print("   BR: ");
//  Serial.println(brMotor);

  topRight.writeMicroseconds(trMotor+1000);
  topLeft.writeMicroseconds(tlMotor+1000);
  bottomLeft.writeMicroseconds(blMotor+1000);
  bottomRight.writeMicroseconds(brMotor+1000);
}

