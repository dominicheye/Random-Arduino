/*
  ControlLED with LCD

  Will control an LED with a potentiometer

  
*/

#include "LiquidCrystal.h"

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);



// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  lcd.begin(16, 2);
  lcd.print("Volts at A0:");
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  float sensorValue = analogRead(A0); //this reads between 0-1023
  // print out the value you read:
  Serial.println(sensorValue);

  lcd.setCursor(0, 1);
  lcd.print((sensorValue/1023) * 5);

  /*
   Say we wanted the voltage to be over 2.5 Volts at the analog pin 0 to turn on the LED
   if 1023 is 5V, then 2.5V should be half of that.
   

   if(sensorValue > 512) {
    digitalWrite(9, HIGH);
   }
   else{
    digitalWrite(9, LOW);
   }*/



  

  /*
    To change the brightness of LED the PWM pin will control that. 
    0 means it is always 0V. 255 means alway 5V. in between is where
    brightness can change because it controls the duty cycle.*/
   

  analogWrite(9, sensorValue/4);



  delay(30);        // delay in between reads for stability
}
