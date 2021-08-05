// Constants
const int enableBridge1 = 2;
const int MotorForward1 = 3;
const int MotorReverse1 = 5;
const int enableEM = 7;
// Variables
int Power = 255; //Motor velocity between 0 and 255

void setup() {
  pinMode(MotorForward1, OUTPUT);
  pinMode(MotorReverse1, OUTPUT);
  pinMode(enableBridge1, OUTPUT); 
  pinMode(enableEM, OUTPUT);
}

void loop() {
  digitalWrite(enableBridge1, HIGH); // Enable Hbridge 1

  // Rotate forward for 2 seconds
  analogWrite(MotorForward1, Power);
  analogWrite(MotorReverse1, 0);
  delay(2000);

  //turn on electromagnet
  digitalWrite(enableEM, HIGH); 
  
  // Rotate backward for 3 seconds
  analogWrite(MotorForward1, 0);
  analogWrite(MotorReverse1, Power);
  delay(3000);

  //Stop motor for 1 second
  analogWrite(MotorForward1, 0);
  analogWrite(MotorReverse1, 0);
  digitalWrite(enableBridge1, LOW);
  delay(1000);

  //turn off electromagnet
  digitalWrite(enableEM, LOW); 
  delay(500); //delay for .5 sec
}
