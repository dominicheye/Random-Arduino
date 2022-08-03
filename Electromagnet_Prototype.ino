/******************************************
            Electromagnet_Prototype

This program will be a test for the electromagnet 
PID controlled platform. The program will control the MOSFET 
that allows current to go through the electromagnet. The 
hardest tasks for this program to capture is the 
PWM frequency (because too high or too low results in no control
of the electromagnet)

The PWM frequency for full control will need to be found,
and then from there the PID constants will need to be found as well.
******************************************/

#define   HE_SENSOR     3
#define   POT_READ      2
#define   PWM_PIN       1
#define   KP            1       //This will change
#define   KI            1       //This will change
#define   KD            1       //This will change
#define   SLEEP_TIME    10      //This will change
#define   DEFAULT_PWM   150     //This will change
#define   LOWER_BOUND   0
#define   UPPER_BOUND   1023


//declare all necessary variables

float output, error, derivative, integral;
float error_prior = 0, integral_prior = 0; 

float HE_read, new_PWM, goal;

void setup() {
  pinMode(HE_SENSOR, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(POT_READ, INPUT);

  goal = analogRead(POT_READ);
  
  //set PWM speed initially here and wait 250 milliseconds
  analogWrite(PWM_PIN, DEFAULT_PWM);
  delay(50);
}

void loop() {
  goal = analogRead(POT_READ);
  HE_read = analogRead(HE_SENSOR);
  new_PWM = PID_Method(HE_read, goal);
  analogWrite(PWM_PIN, (int)new_PWM);
}



float PID_Method(float input, float goal){
  error = goal - input;
  integral = integral_prior + error * SLEEP_TIME;
  derivative = (error - error_prior)/SLEEP_TIME;
  output = KP *error + KI *integral + KD *derivative;

  error_prior = error;
  integral_prior = integral;
  delay(SLEEP_TIME);

  if(output < LOWER_BOUND)
    output = LOWER_BOUND;

  if(output > UPPER_BOUND)
    output = UPPER_BOUND;
      
  return output;
}
