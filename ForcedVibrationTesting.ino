/*
   ForcedVibrationTesting
   Author: Josh Bamberger
   Date: 2-10-2019

   Description:
   This program will be used for the forced vibration testing of the wing. It should be able to control the amplitude and frequency of oscillation for the motors as well as controlling when the
   motors start and stop with the two buttons on the chipkit shield. It is designed to work with the breadboard setup and to allow the use of the accellerometer and Serial with the arduino

*/

//For now softPWMServo.h will be used as the servo.h library doesnt work with the WIFIRE(There is a patch that has come out to address this)
#include <SoftPWMServo.h>
#include <string.h>
#include <Motor.h>

//**************Accellerometer Initialization
#include "I2Cdev.h"
#include "MPU6050.h"

#define TIME_MSG_LEN  11   // time sync to PC is HEADER and unix time_t as ten ascii 
#define TIME_HEADER  255   // Header tag for serial time sync message

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 45
bool blinkState = false;

//**********INITIALIZATION*******************
//Connection Pins
const int signalPin = 33; // GPIO 33 
const int redButton = 32;//GPIO 32
const int whiteButton = 31;//GPIO 31
const int potPin = 14;//AIN 0 Potentiometer Pin
int ledPin = 45;

//Motors
const int numberMotors = 4;
Motor motors[numberMotors];

//Set the number of values in the template oscilation
//Length of precalculated sine array for retrieval while running
const int nSamplesOsc = 400;
float singleSine[nSamplesOsc];
float thrusts[4];
int currSteps[numberMotors] = {0,nSamplesOsc/2,0,nSamplesOsc/2};//This is where the phase offset between all of the motors is set


//Printing Params
int numDecPts = 2;
int microssteps;//microseconds per step
unsigned long currTime;
unsigned long startTime;
unsigned long oscStartTime;
unsigned long stepSize = 10000;//This will control the frequency of data collection

//Current State Variables **** Likely will not need most of these for the forced vibration testing
bool armed = false;
bool armCommand = false;
bool disarmCommand = false;
bool idling = true;
bool idlingLast = false;
bool armPushedLast = false;//this is a bool that stores whether or not the arm button was pushed last time
bool amplPushedLast = false;//This does the same as above for the amplitude switching button
unsigned long armDepressedTime;//This stores the time of when the current arm button was depressed
//Dont need one for the other button because there is not a time intensive part to it.

//Test variables
float verticalOffset = 45;
//float minFreq = 1;
//float maxFreq = 10;
float frequency = 9.75;
int numAmpl = 8;
//float amplitudes[8] = {5,10,15,20,25,30,35,40};
float amplitude = 10;
int currAmplitude = 0;

//UI Options
int longPressThresh = 3000;//Time for a press to be registered as a long press in ms

//Printing out the values
void serialThrustAcc(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float thrusts[4]){
  //This function will send all of the accellerometer data as well as the thrust values back to the computer over serial.
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(thrusts[0]); Serial.print(",");
  Serial.print(thrusts[1]); Serial.print(",");
  Serial.print(thrusts[2]); Serial.print(",");
  Serial.println(thrusts[3]);
}


//Also not sure if the flash functions will be needed
void flash(int times,int ledPin){
  
  for(int i = 0; i < times; i++){
    digitalWrite(ledPin,HIGH);
    delay(500);
    digitalWrite(ledPin,LOW);
    delay(250);
  }
  
}

void fastflash(int times,int ledPin){
  for(int i = 0; i < times; i++){
    digitalWrite(ledPin,HIGH);
    delay(1);
    digitalWrite(ledPin,LOW);
    delay(1);
  }
}


void setup() {
  // **********Accellerometer Setup
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
   //Initialize Serial
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setXAccelOffset(-206);
  accelgyro.setYAccelOffset(-1897);
  accelgyro.setZAccelOffset(1217);
  accelgyro.setXGyroOffset(253);
  
  accelgyro.setYGyroOffset(-25);
  accelgyro.setZGyroOffset(-22);
  
  //setup led
  pinMode(ledPin,OUTPUT);

  //Set the pins for all of the motors
  motors[0].pin = 4;
  motors[1].pin = 5;
  motors[2].pin = 6;
  motors[3].pin = 7;
    
  //Initialize the button
  pinMode(whiteButton, INPUT);
  pinMode(redButton, INPUT);

 

  //Create the single sine array
  for(int i = 0; i < nSamplesOsc; i++){
  //Create the singleSine
  singleSine[i] = sin(2*(float)i*PI/(float)nSamplesOsc);
  }

}// end setup

void loop() {
  //Outline for the Loop
  /*
   * First, get the amplitude frequency and vertical offset for the oscillation over serial
   * White button push to arm
   * Go through the arming sequence
   * Red button starts the accellerometer
   * The accellerometer starts reading(If we have a lot of issues with it reading zeros this could be moved up above the arming so that we dont have to wait so long)
   * Keep sending the thrust input and accellerometer data inputs back to the computer until the motors are stopped at the end
   * White Button starts the oscillation
   * Start the oscialltion
   * Run the oscillation until the red button is pushed
   * Stop sending acceleration data when the white button is pushed
   */

   //Get the amplitude,frequency, and vertical offset over serial **Tested
   Serial.println("Amplitude(Throttle Percent):");
   while(!Serial.available()){};//Wait for the user response
   amplitude = (float) Serial.parseInt();//Set the amplitude
   Serial.read();
   /*
   Serial.println("Frequency(Hz):");
   while(!Serial.available()){};//Wait for the user response
   frequency = (float) Serial.parseInt();//Set the amplitude
   Serial.read();
   */
   Serial.println("Vertical Offset(Throttle Percent):");
   while(!Serial.available()){};//Wait for the user response
   verticalOffset = (float) Serial.parseInt();//Set the amplitude
   Serial.read();

   //Set the microssteps
   microssteps = (int)(1000000/((float)nSamplesOsc*frequency));

   //Wait for the white button to be pushed ***Tested
   while(!digitalRead(whiteButton)){};
   //while(!Serial.available()){};
   //Serial.read();

   Serial.println("Arming Motors");

   //Go through the arming sequence **Tested(Not on hardware)
   setAllMotors(motors, 0, numberMotors);
   delay(2000);
   flash(1,ledPin);
   setAllMotors(motors, 100, numberMotors);
   delay(5000);
   flash(2,ledPin);
   setAllMotors(motors, 0, numberMotors);
   delay(5000);
   flash(3,ledPin);
   setAllMotors(motors, 10, numberMotors);
   delay(5000);
   flash(4,ledPin);
   setAllMotors(motors, 40, numberMotors);
   delay(1000);
   flash(5,ledPin);
   setAllMotors(motors, 60, numberMotors);
   setAllMotors(motors, 17, numberMotors);//Ideally this will be changed to 0 but not sure if that will work or disarm the motor
   thrusts[0] = 17;
   thrusts[1] = 17;
   thrusts[2] = 17;
   thrusts[3] = 17;
   Serial.println("Motors Armed");
   

   //Wait for the red button to start the accellerometer
   while(!digitalRead(redButton)){};//wait for the red button to be pushed
   //while(!Serial.available());
   //Serial.read();
   
   startTime = micros();//Get the current time
   currTime = startTime;
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

   //Wait for the white button while sending all of the thrust and accellerometer values
   while(!digitalRead(whiteButton)){
   //while(!Serial.available()){
      if(micros() > (currTime + stepSize)){
         //Step the time and send the accelleration and thrust values
         accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
         serialThrustAcc(ax,ay,az,gx,gy,gz,thrusts);
         currTime += stepSize;
         fastflash(1,ledPin);
      };
   };
   Serial.read();


   //Once the red button has been pushed the oscillation should start
    oscStartTime = micros();

    //Run the osciallation until the red button is pushed
   while(!digitalRead(redButton)){
   //while(!Serial.available()){
    /*
      if(Serial.available()){
        frequency = Serial.parseFloat();
        //Set the microssteps
        microssteps = (int)(1000000/((float)nSamplesOsc*frequency));
        Serial.read();
      }
      */
      //Check if the reporting time has passed
      if(micros() > (currTime + stepSize)){
         //Step the time and send the accelleration and thrust values
         accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
         serialThrustAcc(ax,ay,az,gx,gy,gz,thrusts);
         currTime += stepSize;
      };
      if(micros() < oscStartTime + microssteps){
            for(int i = 0; i < numberMotors; i++){
              thrusts[i] = (singleSine[currSteps[i]]*amplitude+verticalOffset);
              setMotor(motors,thrusts[i],i);
            }
          }else{
            oscStartTime = oscStartTime + microssteps;
            //iterate all of the current steps;
            for(int i = 0; i <  numberMotors; i++){
              //Change the currSteps
              currSteps[i] += 1;
              currSteps[i] = currSteps[i] % nSamplesOsc;
            }
          }//end if 
    }//End while
    Serial.read();
    //turn all motors off
    setAllMotors(motors, 0, numberMotors);
    thrusts[0] = 0;
    thrusts[1] = 0;
    thrusts[2] = 0;
    thrusts[3] = 0;

    //Wait for the white button while sending accellerometer values
   while(!digitalRead(whiteButton)){
   //while(!Serial.available()){
      if(micros() > (currTime + stepSize)){
         //Step the time and send the accelleration and thrust values
         accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
         serialThrustAcc(ax,ay,az,gx,gy,gz,thrusts);
         currTime += stepSize;
      };
   };
   Serial.read();

   Serial.println("*****************************************");
    
     //testing
     Serial.println(amplitude);
     Serial.println(frequency);
     Serial.println(verticalOffset);

   

/*
  if(digitalRead(armButton)){
    flash(1,ledPin);
  }

  //Get the frequency from the potentiometer
  currentFreq = floatMap(analogRead(potPin),0,4096,minFreq,maxFreq);
  microssteps = (int)(1000000/((float)nSamplesOsc*currentFreq));
  //Serial.println(microssteps);
  //microssteps = 5000;
  

  //Check for arming buttonpush
  if(digitalRead(armButton) && !armPushedLast){
    flash(2,ledPin);
    armPushedLast = true;
    armDepressedTime = millis();
  }else if(!digitalRead(armButton) && armPushedLast){
    armPushedLast = false;
    //do a time check to see if it was a short or a long button press
    if( (millis()-armDepressedTime) > longPressThresh){
      //Long button press
      if(!armed){
        armCommand = true;
        flash(2,ledPin);
      }else{
        disarmCommand = true;
      }//end armed if
    }else {
      //short button push
      idling = !idling;//Toggle the idling of the motor
      startTime = micros();
    }
  }else if(!digitalRead(armButton)){
    armPushedLast = false;
  }//end if

  //Check for the other button push
  if(digitalRead(amplButton) && !amplPushedLast){
    amplPushedLast = true;
  }else if(!digitalRead(amplButton) && amplPushedLast){
    amplPushedLast = false;
    //activate the amplitude switch
    currAmplitude = (currAmplitude+1) % numAmpl;
    //This will be the index variable that will be used when calculating the power settings of the motor
  }

  //Do whatever motor stuff is necessary based on the button push

  //If the arm command has been sent
  if(armCommand){
    //Reset the armCommand to 0 so that it doesnt try to do it the next time through the loop
    armCommand = false;
    setAllMotors(motors, 0, numberMotors);
    delay(2000);
    flash(1,ledPin);
    setAllMotors(motors, 100, numberMotors);
    delay(5000);
    flash(2,ledPin);
    setAllMotors(motors, 0, numberMotors);
    delay(5000);
    flash(3,ledPin);
    setAllMotors(motors, 10, numberMotors);
    delay(5000);
    flash(4,ledPin);
    setAllMotors(motors, 40, numberMotors);
    delay(1000);
    flash(5,ledPin);
    setAllMotors(motors, 60, numberMotors);
    startTime = micros();
    armed = true;
  }

  //Check for the disarm command
  if(disarmCommand){
    disarmCommand = false;
    setAllMotors(motors,0,numberMotors);
    armed = false;
    flash(4,ledPin);
  }

  //Check whether it is armed/idlinig
  if(armed){
    if(idling){
      //if idling set all of the motors to their idle speed
      setAllMotors(motors, 17, numberMotors);
      idlingLast = true;
    }else{
      if(idlingLast){
      }
      idlingLast = false;
      //This is where the motors need to be set to their oscillation speed.
      if(micros() < startTime + microssteps){
        for(int i = 0; i < numberMotors; i++){
          setMotor(motors,(singleSine[currSteps[i]]*amplitudes[currAmplitude]+verticalOffset),i);
          /*
          Serial.println("Curr Step");
          Serial.println(currSteps[i]);
          Serial.println("Amplitude");
          Serial.println(amplitudes[currAmplitude]);
          Serial.println("currAmplitude");
          Serial.println(currAmplitude);
          Serial.println(singleSine[currSteps[i]]*amplitudes[currAmplitude]+verticalOffset);
          
          
        }
      }else{
        startTime = startTime + microssteps;
        //iterate all of the current steps;
        for(int i = 0; i <  numberMotors; i++){
          //Change the currSteps
          currSteps[i] += 1;
          currSteps[i] = currSteps[i] % nSamplesOsc;
          //Serial.println(currSteps[i]);
        }
      }//end if 
    }
    
  }
  */
  
  
}// end loop
