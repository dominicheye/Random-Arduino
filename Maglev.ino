/*****************************************************************************
Maglev Driving Program

Created by Dominic Heye

This program will drive a maglev train that has 3 electromagnets that will push 
and pull their way forward by switching between a north and south pole. This is 
done by switching the current going through these electromagnets. This is easily
done through H-Bridges, and we purchased an Arduino shield from Adafruit that can
control 4 motors through H-Bridges. A library is used with this shield. 


This program will operate in these steps:

1. Declare any needed variables and objects

2. Read Hall Effect Sensors

3. 
******************************************************************************/


/***********************************

      #include
 
***********************************/

#include <Adafruit_MotorShield.h> 
#include <Wire.h> 


/***********************************

      Constants
 
***********************************/

//threshold values to decide when to turn off EM
//when these are too close to 120 (middle value), they will slow the maglev.
//when they are too far, they won't push/pull enough, slowing the maglev.
#define HE_LOW 80         
#define HE_HIGH 160  



/***********************************

      Variables
 
***********************************/

const int hesPin[3] = {0, 1, 2};            //pins for the 4 hall effect sensor readings
const int hesOffset[3] = {0, 0, 0};         //offset to let each one have a center of 120
int hesValue[3];                            //contains hall effect sensor values
int hesPrevValue[3];                        //contains previous hall effect sensor values
enum magField {EM_OFF, EM_NORTH, EM_SOUTH}; //new variable that has three options
//NOTE: motors (electromagnets) will run in FORWARD, REVERSE, or RELEASE (OFF). rename these to make life easier
#define EM_OFF RELEASE
#define EM_NORTH FORWARD
#define EM_SOUTH REVERSE

/***********************************

      Set up motors
 
***********************************/
Adafruit_MotorShield Shield = Adafruit_MotorShield(); //declare shield object
//create the electromagnets
Adafruit_DCMotor *EM1 = Shield.getMotor(1);
Adafruit_DCMotor *EM2 = Shield.getMotor(2);
Adafruit_DCMotor *EM3 = Shield.getMotor(3);


/***********************************

      setup()

This function will turn on Serial monitor,
and setup any other necessary features for
the program.
 
***********************************/

void setup() {


  
  }


/***********************************

      EMpower(int)

This function will turn on/off an 
electromagnet, and will control the direction
by doing 
 
***********************************/

void EMpower(int emNum, magField dir) {


  
  }


/***********************************

      readHES()

This function will read the hall effect
sensors and store the results in the
hesValue array. 
 
***********************************/

void readHES() {


  
  }


/***********************************

      decidePower()

This function will use the readings from
hall effect sensors to decide which EM needs to be
turned on/off. Threshhold values will be set
to decide when to turn on/off magnets 
 
***********************************/
void decidePower() {


  
  }



void loop() {
  

  }
