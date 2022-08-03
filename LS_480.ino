#include <Keyboard.h>

//Define pins, so you only need to change them in one spot
#define BUTTON 7

bool runner = 0;

//Put your setup code here, to run once:
void setup() {
  //Begin Serial communication at 9600 baud rate
  Serial1.begin(9600);
  pinMode(BUTTON, INPUT);
  //create an interrupt that will call send_command() whenever the button pin has a rising edge
  attachInterrupt(digitalPinToInterrupt(BUTTON), send_command, RISING);
  //start the Keyboard process
  Keyboard.begin();
}


//called when button is clicked
//will use Serial.write() to send commands
//then will write the data with the keyboard
void send_command() {
  runner = 1;
  return;
}


String toNumChar(int arr[], int sizeArr){
  String returner;
  
  if(arr[0] == 173) {returner += '-';}

  char newletter;

  bool leave = 1;

  for(int i = 1; i < sizeArr && leave == 1; i++){
    if(arr[i] <186 && arr[i] > 175) {
      arr[i] = arr[i] - 128;
      }
    switch(arr[i]) {
      case 48:
        newletter = '0';
        break;
      case 49:
        newletter = '1';
        break;
      case 50:
        newletter = '2';
        break;
      case 51:
        newletter = '3';
        break;
      case 52:
        newletter = '4';
        break;
      case 53:
        newletter = '5';
        break;
      case 54:
        newletter = '6';
        break;
      case 55:
        newletter = '7';
        break;
      case 56:
        newletter = '8';
        break;
      case 57:
        newletter = '9';
        break;
      case 174:
        newletter = '.';
        break;
      case 69:
        leave = 0;
        newletter = 'x';
        break;
      default:
        newletter = 'x';
        break;  
    }
    if(newletter != 'x')
      returner += newletter;
  }
return returner;
}

void loop() {
  if(runner){
    
  int pkneg[14];
  int pkpos[14];
  int units[4];

  String pknegS,pkposS,unitsS;
  
  Serial1.write('P');
  Serial1.write('K');
  Serial1.write('P');
  Serial1.write('O');
  Serial1.write('S');
  Serial1.write('?');
  Serial1.write('\r');
  Serial1.write('\n');

  delay(100);
  
  if(Serial1.available()) {
     for(int i = 0; i < 14; i++) {pkpos[i] = Serial1.read();}
  }

  pkposS = toNumChar(pkpos, 14);


  
  Serial1.write('P');
  Serial1.write('K');
  Serial1.write('N');
  Serial1.write('E');
  Serial1.write('G');
  Serial1.write('?');
  Serial1.write('\r');
  Serial1.write('\n');

  delay(100);

  
  if(Serial1.available()) {
     for(int i = 0; i < 14; i++) {pkneg[i] = Serial1.read();}
   }

  pknegS = toNumChar(pkneg, 14);

  Keyboard.print(pknegS);
  Keyboard.print('\t');
  Keyboard.print(pkposS);
  Keyboard.print('\n');
  runner =0;
  }
}
