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
	if(!accel.begin()){
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
		pinMode(INTERNAL, OUTPUT);
		while(1){
			digitalWrite(INTERNAL, HIGH);
			delay(1000);
			digitalWrite(INTERNAL, LOW);
			delay(1000);
		}
	}
	if(!mag.begin()){
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
		pinMode(INTERNAL, OUTPUT);
		while(1){
			digitalWrite(INTERNAL, HIGH);
			delay(1000);
			digitalWrite(INTERNAL, LOW);
			delay(1000);
		}
	}
}

float Gyro::getPitchOrientation(){
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation))
		return orientation.roll;
	else
		return -1;
  
}

float Gyro::getRollOrientation(){
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation))
		return orientation.pitch;
	else
		return -1;
}

float Gyro::getYawOrientation(){
	mag.getEvent(&mag_event);
	if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
		return orientation.heading;
	else
		return -1;
}

float Gyro::getAccelX(){
	accel.getEvent(&accel_event);
	return accel_event.acceleration.x;
}

float Gyro::getAccelY(){
	accel.getEvent(&accel_event);
	return accel_event.acceleration.y;
}

float Gyro::getAccelZ(){
	accel.getEvent(&accel_event);
	return accel_event.acceleration.z;
}