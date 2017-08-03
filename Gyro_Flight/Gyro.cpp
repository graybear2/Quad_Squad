#include "Gyro.h"


Gyro::Gyro(){
  this->initSensors();
}
void Gyro::initSensors()
{
  /* Assign a unique ID to the sensors */
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

float Gyro::getPitch(){
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation))
		return orientation.roll;
  else
    return -1;
  
}

float Gyro::getRoll(){
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation))
		return orientation.pitch;
  else
    return -1;
}

float Gyro::getYaw(){
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    return orientation.heading;
  else
    return -1;
}
