/*****************************************************************************
Maglev Driving Program

created by Dominic Heye

This program will drive a maglev train that has 3 electromagnets that will push 
and pull their way forward by switching between a north and south pole. This is 
done by switching the current going through these electromagnets. This is easily
done through H-Bridges, and we purchased an Arduino shield from Adafruit that can
control 4 motors through H-Bridges. A library is used with this shield. 
******************************************************************************/

  
/***********************************

      #include
 
***********************************/

#include <Adafruit_MotorShield.h> 
#include <Wire.h> 
#include <RH_ASK.h>
#include <SPI.h> 


/***********************************

      Constants
 
***********************************/

//threshold values to decide when to turn off EM
//when these are too close to 120 (middle value), they will slow the maglev.
//when they are too far, they won't push/pull enough, slowing the maglev.
//60 is a good low and 280 is a good high
#define HE_LOW_1 120 //140
#define HE_HIGH_1 235 //240
#define HE_LOW_2 110 //90
#define HE_HIGH_2 310 //250
#define HE_LOW_3 120 
#define HE_HIGH_3 255
#define MOTOR_SPEED 255 //between 0-255 (min-max)



/***********************************

      Variables
 
***********************************/

const int hesPin[3] = {3, 0, 1};            //pins for the 4 hall effect sensor readings
const int hesOffset[3] = {0, 0, 0};         //offset to let each one have a center of 120
int hesValue[3];                            //contains hall effect sensor values
int hesPrevValue[3];                        //contains previous hall effect sensor values
//NOTE: motors (electromagnets) will run in FORWARD, REVERSE, or RELEASE (OFF). rename these to make life easier
#define EM_OFF RELEASE
#define EM_NORTH FORWARD
#define EM_SOUTH BACKWARD


RH_ASK driver; //set up driver to communicate with main board

//setup variables only needed in loop()
uint8_t buf[3];
uint8_t buflen = sizeof(buf);
const char onn[4] = "onn";
const char off[4] = "off";
bool onState = false;

/***********************************

      Set up motors
 
***********************************/
Adafruit_MotorShield Shield = Adafruit_MotorShield(); //declare shield object
//create the electromagnets
Adafruit_DCMotor *EM1 = Shield.getMotor(2);
Adafruit_DCMotor *EM2 = Shield.getMotor(3);
Adafruit_DCMotor *EM3 = Shield.getMotor(4);


/***********************************

      setup()

This function will turn on Serial monitor,
and setup any other necessary features for
the program.
 
***********************************/

void setup() {
  //begin serial monitor for debugging
  Serial.begin(9600);

  Shield.begin();
  
  //set motor speed for all three motors
  EM1->setSpeed(MOTOR_SPEED);
  EM2->setSpeed(MOTOR_SPEED);
  EM3->setSpeed(MOTOR_SPEED);


  EM1->run(RELEASE);
  EM2->run(RELEASE);
  EM3->run(RELEASE);

  /*if (!driver.init())
    Serial.println("init failed");*/
  }


/***********************************

      EMpower(int)

This function will turn on/off an 
electromagnet, and will control the direction
by using North/South
 
***********************************/

void EMpowerNorth(int emNum) {

  switch(emNum) {
    case 0: 
      EM1->run(FORWARD);
      Serial.print("\t2");
      return;
    case 1:
      EM2->run(FORWARD);
      Serial.print("\t5");
      return;
    case 2:
      EM3->run(FORWARD);
      Serial.println("\t8");
      return;
    }

  }

void EMpowerSouth(int emNum) {

  switch(emNum) {
    case 0: 
      EM1->run(BACKWARD);
      Serial.print("\t0");
      return;
    case 1:
      EM2->run(BACKWARD);
      Serial.print("\t3");
      return;
    case 2:
      EM3->run(BACKWARD);
      Serial.println("\t6");
      return;
    }

  }

void EMpowerOff(int emNum) {

  switch(emNum) {
    case 0: 
      EM1->run(RELEASE);
      Serial.print("\t1");
      return;
    case 1:
      EM2->run(RELEASE);
      Serial.print("\t4");
      return;
    case 2:
      EM3->run(RELEASE);
      Serial.println("\t7");
      return;
    }

  }


/***********************************

      readHES()

This function will read the hall effect
sensors and store the results in the
hesValue array. 
 
***********************************/

void readHES() {
  int offTrack = 0;
   
  for(int p=0; p<3;p++){
    hesValue[p] = analogRead(hesPin[p]);
    Serial.print(hesValue[p]);
    Serial.print("\t");
    if(hesValue[p] > 125 && hesValue[p] < 135)
        offTrack++;
    }
  //Serial.print("\n"); 
  if(offTrack == 3) {
    allOFF();
    }
  
  
  }


/***********************************

      decidePower()

This function will use the readings from
hall effect sensors to decide which EM needs to be
turned on/off. Threshhold values will be set
to decide when to turn on/off magnets 
 
***********************************/

void decidePower() {
int HE_LOW, HE_HIGH;
  for(int p=0; p<3;p++) {
    switch(p) {
      case 0: 
        HE_LOW = HE_LOW_1;
        HE_HIGH = HE_HIGH_1;
        break;
      case 1: 
        HE_LOW = HE_LOW_2;
        HE_HIGH = HE_HIGH_2;
        break;
      case 2: 
        HE_LOW = HE_LOW_3;
        HE_HIGH = HE_HIGH_3;
        break;
    }
    if(hesValue[p] < HE_LOW ) {
      EMpowerSouth(p);
      }
    else if(hesValue[p] > HE_HIGH ){
      EMpowerNorth(p);
      }
    else {
      EMpowerOff(p);
      }
    }
  
  }



/***********************************

      allOFF()

This function will turn all the electromagnets
off. called if all three hall effect sensors are
around 120 (meaning off the track).
 
***********************************/

void allOFF() {
  //Serial.println("allOFF() is being run");
  EM1->run(RELEASE);
  EM2->run(RELEASE);
  EM3->run(RELEASE);
  }

/***********************************

      loop()

This function will read the incoming signal,
then if it is saying to be on then run the maglev code.
if off then wait until on.

maglev code will call readHES(), then
decidePower(). 
 
***********************************/

void loop() {  
  /*if (driver.recv(buf, &buflen)) {
    // Message with a good checksum received, dump it.
    Serial.print("Message: ");
    Serial.println((char*)buf);   

    if(onn == (char*)buf)
      onState = true;
      
    else 
      onState = false;
    }*/
  onState = true;
  if(onState == true) {
    readHES();
    decidePower();
    }
  }
