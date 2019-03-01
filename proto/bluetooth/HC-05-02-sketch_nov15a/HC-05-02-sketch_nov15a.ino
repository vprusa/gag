#include <SoftwareSerial.h>

//set ports
// state pin
#define rx 3 // => HC05: rx
#define tx 2 // => HC05: tx
// GND pin
// 5V pin
#define cmd 9 //EN pin

SoftwareSerial BTserial(rx, tx); // RX | TX of Arduino

char reading = ' ';

// BTconnected will = false when not connected and true when connected
boolean BTconnected = false;

// The setup() function runs once each time the micro-controller starts
void setup()
{
  // start serial communication with the serial monitor on the host computer
  
  // set input through EN pin
  pinMode(cmd, OUTPUT);
  digitalWrite(cmd, HIGH);

  //Serial turns on in 1 second. Be patient and wait.
  delay(1000);

  // wait until the HC-05 has made a connection
  while (!BTconnected)
  {
    if (digitalRead(cmd) == HIGH) { BTconnected = true; };
  }

  Serial.begin(9600);
  Serial.println("HC-05 is now connected");
  Serial.println("");

  // Start serial communication with the bluetooth module
  // HC-05 default serial speed: 9600 or 38400
  Serial.println("Enter AT commands:");
  BTserial.begin(38400);  // HC-05 default speed in AT command mode
}
void loop()
{
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTserial.available())
  {
    reading = BTserial.read();
    Serial.write( reading );
  }

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
  {
    reading = Serial.read();
    BTserial.write( reading );
  }
}

