/*
 * Title: Motor
 * Author: Josh Bamberger
 * Date: 9-23-2018
 * Description: This struct will hold all of the information specific to each motor for the tests
  */
#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>


//This struct will be an array for holding all of the information for the different motors.
struct Motor: public Servo {
  //Variables for the sine wave
  float a;//amplitude
  float w;//angular freq
  float o;//Phase Shift
  float b;//vertical shift
  int pin;
};

//Voltage and Current Pins
struct sensorPin {
  int current;
  int voltage;
};

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  //modification of the built in map function to deal with floats
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}// end floatMap

void setMotor(Motor motor[], float throttle,int whichMotor){
  //do any rescaling that is necessary
  motor[whichMotor].write(floatMap(throttle,0,100,89,179));
}

void setAllMotors(Motor motors[], float throttle, int numberMotors){
  //This will set all of the motors to a single value
  for(int i = 0; i < numberMotors; i++){
    //set the individual motor
    setMotor(motors,throttle,i);
  }
}



#endif
