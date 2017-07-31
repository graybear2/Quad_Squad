#include "gyro.h"

gyro::gyro(){
  
}
void gyro::initSensors()
{
  dof   = Adafruit_9DOF();
  accel = Adafruit_LSM303_Accel_Unified(30301);
  mag   = Adafruit_LSM303_Mag_Unified(30302);
	if(!accel.begin())
	{
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
		while(1);
	}
	if(!mag.begin())
	{
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}
}

float gyro::getPitch(){
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation))
	{
		return orientation.roll;
	}
}

float gyro::getRoll(){
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation))
	{
    float ret = orientation.pitch;
		return ret;
	}
}
