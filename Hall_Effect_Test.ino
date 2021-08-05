#include <LiquidCrystal.h>

//define variables
#define HALL_PIN A0 
#define LED_PIN 5
int read1 = 0;
int count = 0;
int timer = 0;
bool on_state = false;
char buffer1[10];
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//this runs once at the beginning
void setup() {
  Serial.begin(9600); //turn on serial monitor
  
  //setup digital pins
  pinMode(HALL_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("GET IN THE OVEN");
  lcd.setCursor(0, 1);
  lcd.print("RPM ");
} //end of setup()

void loop() {
  read1 = digitalRead(HALL_PIN);
  //Serial.println(read1);
  if(read1 == 1) {
    on_state = true;
  }


  if(read1 == 0 && on_state ==true)
    {
     count++;
     on_state = false;
    }

   if(timer % 100 == 0) 
    {
    Serial.println(read1);
    }

    
  delay(1);
  timer = timer + 1;
  if(timer == 1000) {
    timer = 0;
    sprintf(buffer1, "%4d", count * 60  );
    Serial.print("RPM = ");
    lcd.setCursor(0, 1);
    lcd.print("RPM ");
    lcd.print(buffer1);
    Serial.println(count * 60);
    count =0;
  }
} //end of loop()
