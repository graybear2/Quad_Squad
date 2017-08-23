#ifndef __Gyro_h__
#define __Gyro_h__

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

class Gyro{
  public:
    Adafruit_9DOF dof;
    Adafruit_LSM303_Accel_Unified accel;
    Adafruit_LSM303_Mag_Unified   mag;
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
  
    Gyro();
    void initSensors();
    float getPitchOrientation();
    float getRollOrientation();
    float getYawOrientation();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
};

#endif
