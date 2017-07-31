#ifndef __gyro_h__
#define __gyro_h__

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

class gyro{
  public:
    /* Assign a unique ID to the sensors */
    Adafruit_9DOF dof;
    Adafruit_LSM303_Accel_Unified accel;
    Adafruit_LSM303_Mag_Unified   mag;
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
  
    gyro();
    void initSensors();
    float getPitch();
    float getRoll();
};

#endif
