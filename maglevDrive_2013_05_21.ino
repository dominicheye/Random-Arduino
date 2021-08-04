/*maglevDrive.ino
  Written by Team Antipodes (Violet Replicon)
  Terra Nova High School FIRST Tech Challenge team #4529
  www.TheOneRobot.com
  July 15, 2011
  May 20, 2013
  ------------------------------------------------------------------------
  This is the Arduino Sketch program for a 3-phase linear motor controller for Antipodes' remote controlled (RC) maglev model as
  exhibited at the 2013 Bay Area Maker Faire.
  
  The program receives wireless throttle values from Antipodes' remote control.  The remote hardware consists of an Arduino Uno R3,
  an Official Arduino Wireless Shield, an XBee Series 1, and a Snootlab Encoder.  The Arduino Sketch program for the remote
  controller is provided serparately.  The motor conroller hardware consists of another Arduino Uno R3, 2 Sparkfun Ardumoto Motor
  Driver Shields, another Official Arduino Wireless Shield, and another XBee Series 1.
  
  The linear motor consists of 6 solenoids with their axes mounted parallel to the surface of the track and perpendicular to the
  direction of travel.  Each coil generates a magnetic field at both ends of an air core (could be adapted to an iron core) that
  repel and attract 2 sets of permanent magnets arranged along the sides of a channel in the track.  The permanent magnets are
  arranged with opposing poles facing each other across the channel, and alternate orientations along the track's length.  Continuous
  motion is achieved by switching the polarity and strength of the coils.  The 6 coils are divided into 3 separate pairs (phases)
  that are wired in parallel, but with opposing windings.  The only reason for altetnating the direction of the windings has to do
  with our chosen coil spacing of 1-2/3 magnet spacings.  This fact has no effect on the programming and should be ignored for an
  initial understanding of our device.  Instead, consider only the first coil in each phase.
  
  Coil control for each phase is calculated using the voltage (0 to +5v) returned from one of three dedicated, linear, bi-directinal
  Hall Effect Sensors.  The sensor's quiescent voltage (the voltage midway between min and max that corresponds to zero field) is
  subtracted from the raw voltage reading so that polarity corresponds to sign, and absolute value corresponds to field strength.
  The polarity and magnitude are then fed to the two input pins for each motor driver, which then direct the 12v power (up to nearly
  1 amp based on our 13ohm coil pairs) to the coils in the desired direction using H-Bridges.  Because the coils lag the sensor
  by one quarter cycle, the coils are off when adjacent to a magnet (and the sensor is between 2 magnets), and the coils are fully on
  when between two magnets (and the sensor is adjacent to a magnet).  As the train moves, the coils switch polarity as they pass
  alternating polarity track magnets, providing continuous forward acceleration without any additional calculation except unit conversion.
  
  Braking is achieved by driving in reverse until movement direction flips.  Reverse driving is achieved by simply multiplying the sensor
  readings by -1.  Position is calculated by counting magnets, and automatic braking is initiated when the vehicle nears the ends of the
  track.  PID speed control is roughly achieved by applying more coil power or coasting, not braking, to save battery power and for 
  clarity of spectator understanding of the basic propulsion principle (observing the monitoring LEDs reflect the forward wafeform vs.
  random driving and braking).  Our track looks smooth to the eye, but is actually filled with relatively massive magnetic potholes.
  Consequently, slower throttle settings result in full power being applied to exit the potholes and long coasting times on descent.
 (Note:  this program does not sustain D for longer than 1 program loop per half magnet cycle, and this should eventually be fixed).
  
  We failed to calculate real-time (updated every loop) speed reliably.  Instead we calculate it intermittently, but very reliably, six
  times per magnet cycle (each time each sensor value crosses zero) because the distance between magnets is very consistent.  Calculating
  actual speed (distance / time) is unnecessary since the distances are always the same.  Consequently, we used the time interval in lieu
  of speed for speed control.  This simplifies the program, but is a bit unintuitive since larger intervals correspond to slower speeds.
  Movement direction is critically important for accurately tracking the magnet count.  Movement direction is determined by multiplying
  the sign of the largest absolute HES value by the sign of the differential (current value - last value) of the smallest absolute HES
  value.  Many other algorithms were tried and failed to produce correct results throughout the entire cycle at this program's data
  aquisition rate.
  
  The motor controller's XBee receives serial input from the remote's XBee as integer values (after conversion from Bytes to Integers)
  from -5 to 5, corresponding to full speed reverse to full speed forward, as well as a braking integer value of 100.
  
  Experienced programmers will want to type-cast some of the numeric values instead of using more memory intensive data types.
  
  Note:  turning off the LEDs is equivalent to turning off the coils since they are wired in parallel.  LED testing is performed before
  full 12V power is applied to the coils, and that is why many of the functions have names like "ledOn" and "allLedsOff".
*/

//  Global variables
//  ------------------------------------------------------------------------------------------
//  Note: array variables are used to store values for each of the 3 phases.  Phase A = 0, B=1, C=2.
//  If you are unfamiliar with array variables this is a simple application and an easy one to learn on.  For example,
//  "long hesVal[3] = {};" is the variable declaration for storing three Hall Effect Sensor readings of type LONG (nevermind for
//  the moment that values will not exceed the INT range), and sets all three initial values to zero (by default).  This could be
//  equivalently written as "long hesVal[3] = {0, 0, 0};".  Refering to the values is easy:  hesVal[0] stores the Hall Effect Sensor
//  value for the first phase (which we label 'A' on our physical model), hesVal[1] stores the second sensor value (phase B), and
//  hesVal[2] stores the third sensor value (phase C).  This simplifies the program in certain places in particular.  For example:
//
//    digitalWrite(dirPin[phase], ledDir);
//
//  without using arrays might need to be written as:
//
//    switch(phase) {
//      case 0:
//        digitalWrite(dirPin0), ledDir);
//        break;
//      case 1:
//        digitalWrite(dirPin1), ledDir);
//        break;
//      case 2:
//        digitalWrite(dirPin2), ledDir);
//    }
//
//  as well as having to declare the three variables dirPin0, dirPin1, and dirPin2, instead of the single array variable dirPin[3].

const int powerPin[3] = {3, 11, 5};  // digital pin 3 controls PWM for phase A, pin 11 for B, and pin 5 (rerouted back to 3 on the upper board) for C
const int dirPin[3] = {12, 13, 8};   // digital pin 12 controls DIR for phase A, pin 13 for B, and pin 8 (rerouted back to 12 on the upper board) for C
const int hesPin[3] = {2, 1, 0};     // analog input pins for Hall Effect Sensor outputs (numbered backward only for ease of wiring)
const int hesOffset[3] = {11, 5, 2}; // calibration offsets that zero the sensors away from magnetic fields
long hesVal[3] = {};                 // Hall Effect Sensor values
long hesLast[3] = {};                // previous Hall Effect Sensor values
int hesLastMagnet[3] = {};           // the HES value the last time a magnet was counted
const long minOnHesVal = 100;        // the minimum threshold for switching on coils
long onHesVal = minOnHesVal;         // the threshold for switching on coils.  This is varied to achieve speed control
long throttle = 0;                   // the speed desired by the user as transmitted by the remote's XBee
long throttleMax = 5;                // the maximum throttle setting (needs to be the same number in this code and in the remote's encoder code)
boolean braked = false;              // tracks whether braking has completed
boolean braking = false;             // tracks whether braking has been initiated
const int brakingDuration = 1000;    // backup measure for switching off braking.  Time since braking initiated (in milliseconds)
int currentMagnet[3] = {};           // the current magnet count
int lastCountMovement[3] = {};       // the direction the vehicle was traveling the last time a magnet was counted
boolean inTrack = false;             // whether or not the vehicle is in the track.  Used to turn off the coils.
int incomingByte;                    // a variable to read incoming serial data into
int incomingInt = 0;                 // initializes the integer translation of the incoming Byte
int maxPhase;                        // the phase with the highest absolute HES value
int minPhase;                        // the phase with the lowest absolute HES value
int phaseRecent;                     // the phase that most recently passed a magnet
long currentTime = 0;                // the current time in milliseconds since reboot
long brakingStartTime = 0;           // the time in milliseconds since braking was initiated
long lastCountTime[3] = {};          // the time in milliseconds since a magnet was counted
float movement = 0;                  // travel direction:  positive numbers are forward, negative backward.  Magnitude is irrelevent.
float brakingStartMovement = 0;      // the movement at the time braking was initiated.  Braking stops when this x movement < 0
const int highBreakMagnet = 8;       // the magnet count past which braking should be initiated when moving forward
const int lowBreakMagnet = 4;        // the magnet count past which braking should be initiated when moving backward
const long slowInterval = 200;       // the slowest desired speed (largest time interval).  Speed target for throttle = 1
const long fastInterval = 40;        // the fastest desired speed (smallest time interval).  Speed target for throttle = 5
long targetInterval;                 // the calculated target interval used in PID speed control
long timeInterval[3] = {2 * slowInterval, 2 * slowInterval, 2 * slowInterval};
                                     // Interval is the time (milliseconds) between magnets.  It is used in lieu of speed since the distances
                                     // between magnets are fixed.  However, lower intervals correspond to faster speeds.  Here it's being
                                     // initialized to a large number so the program understands it is stopped at startup.
long maxHesVal = 0;                  // greatest absolute HES value, but retains sign
long minHesVal = 0;                  // least absolute HES value, but retains sign
const int midHesVal = slowInterval + ((512 - minOnHesVal) / 2);  // uses initial onHesVal
const float kP = 1;                  // Proportional constant used in PID speed control
const float kI = 0.2;                // Integral constant used in PID speed control
const float kD = 1;                  // Differential constant used in PID speed control
const float dampI = 0.66;            // Integral dampening used in PID speed control
long error = 0;                      // the difference between the current and target intervals (speed) used in PID speed control
long integral = 0;                   // the Integral (cummulative sum of errors)


void setup(){
  pinMode(powerPin[0], OUTPUT);      // set to OUTPUT the Arduino pins that will control the motor driver PWM
  pinMode(powerPin[1], OUTPUT);
  pinMode(powerPin[2], OUTPUT);
  pinMode(dirPin[0], OUTPUT);        // set to OUTPUT the Arduino pins that will control the motor driver direction
  pinMode(dirPin[1], OUTPUT);
  pinMode(dirPin[2], OUTPUT);
  Serial.begin(9600);                // initialize serial communication
  for(int i = 0; i < 3; i++){                                  // for each of the 3 phases...
    hesVal[i] = analogRead(hesPin[i]) - (512 + hesOffset[i]);  // get the initial Hall Effect Sensor readings
  }
  for(int i = 0; i < 3; i++){                                  // for each of the 3 phases...
    hesLastMagnet[i] = hesVal[i];                              // tell the program the last time a magnet was counted was at startup
  }                                                            // in order to get valid initial intervals, etc.
}


int byteToInt(byte inByte){
// -----------------------------------------------------------------------------------------------
//  function that converts bytes received by the XBee to integers
// -----------------------------------------------------------------------------------------------
  int inInt;
  if (inByte > 127){
    inInt = inByte - 256;
  }
  else {
    inInt = inByte;
  }
  return inInt;
}


int getThrottle(){
// -----------------------------------------------------------------------------------------------
//  function that reads throttle values received by the XBee
//  Note:  the braking function sets incomingInt to 100 to use subsequent code as-is
// -----------------------------------------------------------------------------------------------
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    incomingInt = byteToInt(incomingByte);  
  }
  return incomingInt;        
}


void getHesValues(){
// -----------------------------------------------------------------------------------------------
// function that reads the voltage output from our linear, bi-directional Hall Effect Sensors.  Voltages range from approximately
// 0 (full South) to +5V (full North) with +2.5V = no field (quiescent).  AnalogRead converts these voltages to values from 0 to
// 1024 with 512 = no field (quiescent).  Subtracting 512 (quiescent) converts these values to a range of -512 to +512 with 0 = 
// no field (quiescent).  Consequently, the sign of the values can be used for determining magnetic field polarity, and the
// magnitude for determining field strength.  Checking for an absolute maximum reading over 100 will determine whether or not the
// car is in the track.  The absolute maximum and minimum readings are also stored 
// -----------------------------------------------------------------------------------------------
  for(int i = 0; i < 3; i++){                // for each of the 3 phases...
    hesLast[i] = hesVal[i];                  // set the current HES values to the previous HES values before... 
    hesVal[i] = analogRead(hesPin[i]) - (512 + hesOffset[i]);    // updating current HES values with new HES sensor readings
  } 
  if (abs(hesVal[0]) > abs(hesVal[1])) {     // if the phase A sensor is closer to a magnet than the phase B sensor...
    maxPhase = 0;                            // temporarily assume that A is the max phase...
    maxHesVal = hesVal[0];                   // and update the maximum HES value...
    minPhase = 1;                            // and temporarily assume that B is the min phase...
    minHesVal = hesVal[1];                   // and update the minimum HES value...
  }
  else {                                     // otherwise...
    maxPhase = 1;                            // temporarily assume that B is the max phase...
    maxHesVal = hesVal[1];                   // and update the maximum HES value...
    minPhase = 0;                            // and temporarily assume that A is the min phase...
    minHesVal = hesVal[0];                   // and update the minimum HES value...
  }
  if (abs(hesVal[2]) > abs(maxHesVal)){      // if the phase C sensor is closer to a magnet than either A or B...
    maxPhase = 2;                            // remember that C is the max phase (leaving the min phase whichever was set above)...
    maxHesVal = hesVal[2];                   // and update the max HES value (leaving min HES value whichever was set above)...
  }
  else if (abs(hesVal[2]) < abs(minHesVal)){ // otherwise if the phase C sensor is further from a magnet than either A or B...
    minPhase = 2;                            // remember that C is the min phase (leaving the max phase whichever was set above)...
    minHesVal = hesVal[2];                   // and update the min HES value (leaving max HES value whichever was set above)...
  }
  if (abs(maxHesVal) > 100) {                // if there are any sensors near a magnet, remember that the vehicle is in the track 
    inTrack = true;
  }
  else {                                     // otherwise remember that the vehicle is not in the track...
    inTrack = false;
    for(int i = 0; i < 3; i++){              // and for each of the 3 phases...
      hesVal[i] = 0;                         // assign 0 to hesVal
    } 
  }
}


void getMovement(){
// -----------------------------------------------------------------------------------------------
//  function that determines which direction the vehicle is moving and assigns this to the global
//  variable "movement".  Movement has type FLOAT because it is later multiplied to produce a
//  FLOAT result (this could be recast instead).  Positive values correspond to forward movement.
//  Negative values correspond to backward movement.  Movement direction is critically important
//  for accurately tracking the magnet count.  Movement direction is determined by multiplying
//  the sign of the largest absolute HES value by the sign of the differential (current value - 
//  last value), which we call "distance" of the smallest absolute HES value.  Many other algorithms
//  were tried and failed to produce correct results throughout the entire magnet cycle at this
//  program's data aquisition rate.
// -----------------------------------------------------------------------------------------------
  long distance[3] = {};                            // variable to store: HES value - previous HES value
  for(int i = 0; i < 3; i++){                       // for each of the 3 phases...
    distance[i] = hesVal[i] - hesLast[i];           // set distance to current HES value - previous HES value
  } 
  switch(maxPhase){                                 
    case 0:                                         // when max HES is A:
      if (minPhase == 1){                           // and min HES is B... 
        if (maxHesVal * distance[1] > 0) movement = 1;  // if HES A x distance B is positive, movement is forward...
        else movement = -1;                         // otherwise movement is backward
      }
      else if (maxHesVal * distance[2] > 0) movement = 1; // otherwise min HES is C, and if HES A x distance C is positive, movement is forward...
      else movement = -1;                           // otherwise movement is backward
      break;
    case 1:                                         // when max HES is B:
      if (minPhase == 0){                           // and min HES is A... 
        if (maxHesVal * distance[0] < 0) movement = 1;  // if HES B x distance A is negative, movement is forward...
        else movement = -1;                         // otherwise movement is backward
      }
      else if (maxHesVal * distance[2] > 0) movement = 1; // otherwise min HES is C, and if HES B x distance C is positive, movement is forward...
      else movement = -1;                           // otherwise movement is backward
      break;
    case 2:                                         // when max HES is C:
      if (minPhase == 0){                           // and min HES is A... 
        if (maxHesVal * distance[0] < 0) movement = 1;  // if HES C x distance A is negative, movement is forward...
        else movement = -1;                         // otherwise movement is backward
      }
      else if (maxHesVal * distance[1] < 0) movement = 1; // otherwise min HES is B, and if HES C x distance B is positive, movement is forward...
      else movement = -1;                           // otherwise movement is backward
  }   
}


void countMagnets(int phase){
// -----------------------------------------------------------------------------------------------
//  function to keep track of the position of each Hall Effect Sensor on the track, and consequently, the vehicle.
//  Note:  magnets are said to have been crossed actually when the next midpoint between magnets have been crossed.
//  There is a danger in inadvertently miscounting magnets when the vehicle changes direction.  The algorithm below accounts for this.
// -----------------------------------------------------------------------------------------------
  long currentTime = millis();                             // temporarily stores the current time for use in this function only
  long checkInterval = currentTime - lastCountTime[phase]; // store how long it has been since this phase's sensor passed a magnet midpoint
  if (checkInterval > timeInterval[phase]) timeInterval[phase] = checkInterval;  // because timeInterval is at least this big. Important for stopped condition.  
  if (hesVal[phase] * hesLastMagnet[phase] < 0) {          // if the sensor has crossed the midpoint between magnets...
    if (lastCountMovement[phase] * movement > 0) {         // and if the vehicle is moving in the same direction as when it last crossed a magnet midpoint...
      hesLastMagnet[phase] = hesVal[phase];                // then update the HES value when the last magnet midpoint was crossed...
      phaseRecent = phase;                                 // and remember that this phase was the one that most recently crossed a magnet...
      timeInterval[phase] = checkInterval;                 // and update this phase's timeInterval (speed)...
      lastCountTime[phase] = currentTime;                  // and remember the current time so future intervals can be calculated...
      if (movement > 0) currentMagnet[phase]++;            // so, add a magnet if moving forward...
      else currentMagnet[phase]--;                         // otherwise movement is backward, so subtract a magnet
    }
    lastCountMovement[phase] = movement;                   // otherwise vehicle has reversed direction.  Update the time, but don't adjust the magnet count
  } 
  if (timeInterval[phase] > slowInterval * 2) timeInterval[phase] = slowInterval * 2;  // don't allow massive intervals to skew Integrals for a long time...
}                                                          // otherwise, and typically, do nothing


void getSpeed(){
// -----------------------------------------------------------------------------------------------
//  function that sets the HES value at which coils will be turned on, effectively adjusting power (a VERY slow PWM)
//  and speed with PID control.
// -----------------------------------------------------------------------------------------------
  int powerP;                             // Proportional speed control (correction proportional to current error)
  int powerI;                             // Integral speed control (correction based on accumulated errors)
  int powerD;                             // Differential speed control (correction based on projected future error)
  int throttleUsed = throttle;            // make a local copy of the throttle value to preserve the actual throttle input
  if (abs(throttleUsed) > throttleMax) throttleUsed = throttleMax;  // eliminates big numbers due to braking throttle of 100
  if (abs(throttleUsed) == throttleMax) onHesVal = minOnHesVal;     // if max throttle, forget speed control to demonstrate max acceleration...
  else {                                  // otherwise throttle is between extremes and speed should be controlled...
    targetInterval = (((slowInterval - fastInterval) * (throttleMax - abs(throttleUsed))) / (throttleMax - 1)) + fastInterval; // target speed
    long lastError = error;               // save current error as previous error before updating current error
    error = timeInterval[phaseRecent] - targetInterval;  // compare the speed of the most recent phase to the desired speed and use it as the error
    if (error <= 0) integral = 0;         // reset Integral when desired speed has been achieved (not using Integral to slow down)
    else integral = (dampI * integral) + error;  // integral calculation (I).  Recent errors are more heavily weighted through dampening.
    powerP = kP * error;                  // proportional calculation (P)
    powerI = kI * (integral + error);     // integral calculation {I}
    powerD = kD * (error - lastError);    // differential calculation {D}
    onHesVal = midHesVal - (powerP + powerI + powerD);  // sets the threshold above which coils are turned on
    if (onHesVal < minOnHesVal) onHesVal = minOnHesVal; // reset the treshold if below the minimum threshold
  }
}


int getPower(int rawValue){
// -----------------------------------------------------------------------------------------------
//  function that returns either 255 or 0 for PWM output to the Sparkfun Ardumoto motor driver boards.
//  These values turn the coil either full on or full off.  Presumably, true PWM could be fed to the motor
//  driver with the proper capacitor connected across the coil.
// -----------------------------------------------------------------------------------------------
  int power;                                  // local variable to be returned from the function
  if (abs(rawValue) > onHesVal) power = 255;  // HES value exceeds the threshold calculated in getSpeed, turn full on
  else power = 0;                             // otherwise turn full off
  return power;
}


int getDir(int rawValue){
// -----------------------------------------------------------------------------------------------
//  function that returns a 0 or 1, as specified to control the direction of the current through the motor outputs.
//  If the product of the HES value and throttle are negative, direction is set to reverse, otherwise direction is set to forward.
// -----------------------------------------------------------------------------------------------
  if (rawValue * throttle < 0) return 0;      // if product of HES value and throttle are negative, reverse...
  else return 1;                              // otherwise forward
}


void ledOn(int phase, int ledDir, int ledPower){
// -----------------------------------------------------------------------------------------------
//  function that writes direction and power to the digital output pins for input to the motor driver shields.
// -----------------------------------------------------------------------------------------------
  digitalWrite(dirPin[phase], ledDir);          // motor direction, 1 = forward, 0 = reverse
  analogWrite(powerPin[phase], ledPower);       // motor power, 255 = full on, 0 = full off    
}


void drive(){
// -----------------------------------------------------------------------------------------------
//  function that writes direction and power to the digital output pins for input to the motor driver shields.
// -----------------------------------------------------------------------------------------------
 for(int i = 0; i < 3; i++){                           // for each of the 3 phases...
   ledOn(i, getDir(hesVal[i]), getPower(hesVal[i]));   // send power to the coils based on HES values, throttle and PID speed control
 } 
}


void brake(){  
// -----------------------------------------------------------------------------------------------
//  function that is the opposite of DRIVE.  Direction is the opposite of current movement.
// -----------------------------------------------------------------------------------------------
 for(int i = 0; i < 3; i++){                                         // for each of the 3 phases...
  ledOn(i, getDir(hesVal[i] * movement * -1), getPower(hesVal[i]));  // same as drive, except in the opposite direction
 } 
}


void allLedsOff(){
// -----------------------------------------------------------------------------------------------
//  function that turns off all the coils.  Direction is irrelevent.
// -----------------------------------------------------------------------------------------------
 for(int i = 0; i < 3; i++){
  analogWrite(powerPin[i], 0);   
 } 
}


void loop(){
// -----------------------------------------------------------------------------------------------
//  the MAIN iterative function that turns off all the coils.
// -----------------------------------------------------------------------------------------------
 getHesValues();                        // read the Hall Effect Sensors
 getMovement();                         // determine which way the vehicle is moving
 for(int i = 0; i < 3; i++){            // for each of the 3 phases...
   countMagnets(i);                     // check the magnets
 } 
 if (inTrack == false){                 // if the vehicle is not in the track...
   allLedsOff();                        // turn off all the coils...
 }
 else{                                  // otherwise...
   throttle = getThrottle();            // get the throttle input from the XBee
   getSpeed();                          // calculate the PID speed values
   if (throttle == 100){                // if braking has been requested by the user or the magnet count...
     if (braking == true){              // and if braking has already been initiated...
       if (movement != brakingStartMovement || millis() - brakingStartTime > brakingDuration) {  // if car has reversed or braking has taken too long, stop.
         braking = false;               // no longer braking...
         braked = true;                 // because vehicle has braked
         allLedsOff();                  // turn off the coils
       }
       else {                           // otherwise...
         brake();                       // continue braking
       }
     }
     else if (braked == true) {         // braking is false and braked is true, so has already braked. Do nothing.
     }
     else {                             // otherwise braking is false and braked is false, so first time braking...
       braking = true;                  // and set braking to true...
       brakingStartTime = millis();     // and remember the time when braking began...
       brakingStartMovement = movement; // and remember the direction the vehicle was traveling when braking was initiated...
       brake();                         // and brake the vehicle
     }
   }
   else if ((currentMagnet[1] > highBreakMagnet && throttle >= 0) || (currentMagnet[1] < lowBreakMagnet && throttle <= 0)) incomingInt = 100; // brake if near the track ends
   else {                               // otherwise user desires driving and not at the end of the track...
     braked = false;                    // so, no longer stopped...
     braking = false;                   // and no longer braking...
     drive();                           // so, drive
   }
 }
}


