#include <Servo.h>
#include <PID_v1.h>

//initialize Servo object
Servo servo;

//these will need to change (FOR PID)
#define PIN_INPUT 0
#define PIN_OUTPUT 9 
#define HALL_PIN 1

//CHANGE THIS TO AMOUNT OF READINGS PER ROTATION
#define RPM_PEGS 4
//CHANGE THIS FOR BETTER PRECISION IN RPM READINGS, BUT LESS OFTEN READINGS
#define RPM_COUNTER 20

//declare functions
//double rpmRead();

//create variables needed for rpm readings
double realRPM, targetRPM = 100;

//create PID variables
double PIDsetpoint, PIDinput, PIDoutput;
double kp=2, ki=5, kd=1; //change these for control 

//initialize PID object
PID motorPID(&PIDinput, &PIDoutput, &PIDsetpoint, kp, ki, kd, DIRECT);



void setup() {
  // attach pin number with specific timings
  servo.attach(9,500,2500);
  //start point for magnets
  servo.write(90);

  //initialize the variables input and setpoint
  PIDinput = rpmRead();
  PIDsetpoint = targetRPM; //this what we want rpm to be

  //turn on the PID
  motorPID.SetMode(AUTOMATIC); 
  }



/*********************************
 * loop will call rpmRead() to get the reading for the RPM
 * it will then set input to the RPM, and then call Compute()
 * 
 * When Compute is finished, the new output will be mapped to
 * degrees where the servo will pull in or out. 
 * 
 *********************************/
void loop() {
  realRPM = rpmRead();

  motorPID.Compute();
  map(PIDoutput, 0, 255, 0, 180);
  servo.write(PIDoutput);
  }



/*********************************
 * rpmRead will read the output from the hall effect vane sensor
 * and whenever a change is read the counter wil increase. the amount of time 
 * with the count can be used to find an rpm
 *********************************/
double rpmRead() {
  double startTime = 0, endTime = 0, timeElapsed = 0; // these are for timing
  int count = 0;
  bool onState = false;

  //have start time set
  startTime = micros();

  //this while loop will repeat itself until count is equal to the RPM_COUNTER,
  //which is specified above. 
  while(count < RPM_COUNTER) {
    if(digitalRead(HALL_PIN) == 1) onState = true;
    if(digitalRead(HALL_PIN) == 0 && onState == true) {
      onState = false;
      count++;
      }
    }
  //once count has reached the right amount, read time into endTime
  endTime = micros();
  timeElapsed = (endTime - startTime) / 1000000; //change in time divided by 10^6 to change from microsec to sec
  
  double countPerMin =  60 * (count / timeElapsed); //divide the count by change in time (sec) and *60 to convert to minutes
  double RPM = countPerMin / RPM_PEGS;
  return RPM;
  }
