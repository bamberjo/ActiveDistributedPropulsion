/*
   ForcedVibrationTesting
   Author: Josh Bamberger
   Date: 10-30-2019

   Description:
   This prograrm will test the motor and the accellerometer running at the same time

*/

//For now softPWMServo.h will be used as the servo.h library doesnt work with the WIFIRE(There is a patch that has come out to address this)
#include <SoftPWMServo.h>
#include <string.h>
#include "Motor.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define TIME_MSG_LEN  11   // time sync to PC is HEADER and unix time_t as ten ascii 
#define TIME_HEADER  255   // Header tag for serial time sync message

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long currTimeAccel;
unsigned long startTimeAccel;
unsigned long stepSizeAccel = 10000;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 45
bool blinkState = false;

//**********INITIALIZATION*******************
//Connection Pins
const int signalPin = 4; // GPIO 33 
const int redButton = 3;//GPIO 32
const int whiteButton = 2;//GPIO 31
unsigned long pulseDuration = 10000;

//Motors
const int numberMotors = 2;
Motor motors[numberMotors];

//Set the number of values in the template oscilation
//Length of precalculated sine array for retrieval while running
const int nSamplesOsc = 100;
float singleSine[nSamplesOsc];
float thrusts[numberMotors];
int currSteps[numberMotors] = {0,50};//This is where the phase offset between all of the motors is set


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
float frequency = 5;
float amplitude = 10;
int currAmplitude = 0;

//UI Options
int longPressThresh = 3000;//Time for a press to be registered as a long press in ms

//Printing out the values
void serialThrustAcc(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float thrusts[4],unsigned long reltime){
  //This function will send all of the accellerometer data as well as the thrust values back to the computer over serial.
}


void setup() {
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(57600);

  motors[0].pin =  5;
  motors[1].pin = 4;

    
  //Initialize the button
  pinMode(whiteButton, INPUT);
  pinMode(redButton, INPUT);

  //Initialize the signal Sender
  pinMode(signalPin,INPUT);

  //Create the single sine array
  for(int i = 0; i < nSamplesOsc; i++){
  //Create the singleSine
  singleSine[i] = sin(2*(float)i*PI/(float)nSamplesOsc);
  }
  Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXAccelOffset(-2600);
    accelgyro.setYAccelOffset(1395);
    accelgyro.setZAccelOffset(1843);
    accelgyro.setXGyroOffset(102);
    
    accelgyro.setYGyroOffset(-18);
    accelgyro.setZGyroOffset(-7);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");

    pinMode(LED_PIN, OUTPUT);
    startTimeAccel = micros();
    currTimeAccel = startTime;

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

   Serial.println("frequency:");
   while(!Serial.available()){};//Wait for the user response
   frequency = (float) Serial.parseFloat();//Set the amplitude
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
   //while(!digitalRead(whiteButton)){};
   //while(!Serial.available()){};
   //Serial.read();

   Serial.println("Arming Motors");

   //Go through the arming sequence **Tested(Not on hardware)
   setAllMotors(motors, 0, numberMotors);
   delay(3000);
   //flash(1,ledPin);
   setAllMotors(motors, 100, numberMotors);
   delay(5000);
   //flash(2,ledPin);
   setAllMotors(motors, 0, numberMotors);
   delay(5000);
   //flash(3,ledPin);
   setAllMotors(motors, 10, numberMotors);
   delay(5000);
   //flash(4,ledPin);
   setAllMotors(motors, 40, numberMotors);
   delay(1000);
   //flash(5,ledPin);
   setAllMotors(motors, 60, numberMotors);
   setAllMotors(motors, 17, numberMotors);//Ideally this will be changed to 0 but not sure if that will work or disarm the motor
   Serial.println("Motors Armed");
 
 /*
   setAllMotors(motors, 0, numberMotors); //this will arm it, but changes some setting and doesnt let the motor spin slowly without stopping
   delay(1500);
   //flash(1,ledPin);
   setAllMotors(motors, 30, numberMotors);
   delay(750);
   //flash(2,ledPin);
   setAllMotors(motors, 15, numberMotors);
   delay(500);
   //flash(3,ledPin);
   setAllMotors(motors, 30, numberMotors);
   delay(200);
   setAllMotors(motors, 40, numberMotors);
   delay(200);
   setAllMotors(motors,50, numberMotors);
   delay(200);
   setAllMotors(motors, 60, numberMotors);
   delay(1000);
   setAllMotors(motors,50, numberMotors);
   delay(1000);
   setAllMotors(motors,45, numberMotors);
   delay(1000);
   */
   
   /*
   startTime = micros();
   while((micros()-startTime) < 50000000)
   {
    setAllMotors(motors, ((float)analogRead(12)/(float)4096)*100,numberMotors);
    Serial.println((float)analogRead(12)/(float)4096*100);
   }
   */

   //Wait for the red button to start the accellerometer
   //while(!digitalRead(signalPin)){
    //Serial.println(digitalRead(signalPin));
    //delay(1);
    //};//wait for the red button to be pushed
   //while(!Serial.available());
   //Serial.read();
   //delay(1000);
   //Serial.println("Triggered");
   
   startTime = micros();//Get the current time
   currTime = startTime;


   //Once the red button has been pushed the oscillation should start
    oscStartTime = micros();
    startTime = micros();//Get the current time
    currTime = startTime;

    //Run the osciallation until the red button is pushed
   while((oscStartTime-startTime) < 10000000){
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.print(currTimeAccel-startTimeAccel); Serial.print(",");
        Serial.print(ax); Serial.print(",");
        Serial.print(ay); Serial.print(",");
        Serial.print(az); Serial.print(",");
        Serial.print(gx); Serial.print(",");
        Serial.print(gy); Serial.print(",");
        Serial.println(gz);
      //Serial.print(analogRead(12));
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
    //Serial.read();
    //turn all motors off
    setAllMotors(motors, 0, numberMotors);

    //Wait for the white button while sending accellerometer values
   //while(!digitalRead(whiteButton)){
   //while(!Serial.available()){
   //};


   
 
  
  
}// end loop
