/*
Read the joystick. Only print values that change.
// http://www.goodliffe.org.uk/arduino/joystick.php

TODO .. like everythin ...
*/
int analogInputPinX   = 27;
int analogInputPinY   = 25;
int digitalInputPin  = 12;
int skipDiff = 70;
int wait = 1000; 
long analogInputValX;
long analogInputValY;
long last_analogInputValX;
long last_analogInputValY;
int digitalInputVal;

void setup()
{
  pinMode(analogInputPinX,   INPUT);
  pinMode(analogInputPinY,   INPUT);
  pinMode(digitalInputPin,   INPUT);
  
  Serial.begin(9600);  // ...set up the serial ouput on 0004 style
}

void loop()
{
  analogInputValX = analogRead(analogInputPinX);
  analogInputValY = analogRead(analogInputPinY);
  digitalInputVal = digitalRead(digitalInputPin);
  if (digitalInputVal != 0) {
    Serial.print("YAY ! Digital is ");
    Serial.print(digitalInputVal);
    Serial.println(" "); // println, to end with a carriage return
  }        
  if (abs(last_analogInputValX - analogInputValX) > skipDiff) {
    Serial.print("Change X to ");
    Serial.print(analogInputValX);    
    Serial.println(" "); // println, to end with a carriage return      
  }
  if (analogInputValX != last_analogInputValX){
    last_analogInputValX = analogInputValX;
  }
  if (abs(analogInputValY - last_analogInputValY) > skipDiff) {
    Serial.print("Change Y to ");
    Serial.print(analogInputValY);    
    Serial.println(" "); // println, Uto end with a carriage return
  }
  if (analogInputValY != last_analogInputValY){
    last_analogInputValY = analogInputValY;
  }

}
   
