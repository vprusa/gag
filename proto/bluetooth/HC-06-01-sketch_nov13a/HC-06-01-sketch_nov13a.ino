/*
Command   Response  Comment
AT  OK  Used to verify communication
AT+VERSION  OKlinvorV1.8  The firmware version (version might depend on firmware)
AT+NAMExyz  OKsetname   Sets the module name to “xyz”
AT+PIN1234  OKsetPIN  Sets the module PIN to 1234
AT+BAUD1  OK1200  Sets the baud rate to 1200
AT+BAUD2  OK2400  
ASets the baud rate to 2400
AT+BAUD3  OK4800  Sets the baud rate to 4800
AT+BAUD4  OK9600  Sets the baud rate to 9600
AT+BAUD5  OK19200   Sets the baud rate to 19200
AT+BAUD6  OK38400   Sets the baud rate to 38400
AT+BAUD7  OK57600   Sets the baud rate to 57600
AT+BAUD8  OK115200  Sets the baud rate to 115200
AT+BAUD9  OK230400  Sets the baud rate to 230400
AT+BAUDA  OK460800  Sets the baud rate to 460800
AT+BAUDB  OK921600  Sets the baud rate to 921600
AT+BAUDC  OK1382400   Sets the baud rate to 1382400
*/
 
#include <SoftwareSerial.h>
//SoftwareSerial BTserial(2, 3); // RX | TX
//SoftwareSerial BTserial(10, 9); // RX | TX
//SoftwareSerial BTserial(0, 1); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 2. 
// Connect the HC-06 RX to the Arduino TX on pin 3 through a voltage divider.
// 
// Wokring test
#define RX 1
#define TX 0
SoftwareSerial mySerial(RX, TX); // RX, TX
 void waitForResponse() {
    delay(1000);
    while (mySerial.available()) {
      Serial.write(mySerial.read());
    }
    Serial.write("\n");
}

// http://www.ablab.in/how-to-know-the-change-the-baud-rate-of-a-hc-06-bluetooth-module-with-avr-atmega32-microcontroller/
// https://electronics.stackexchange.com/questions/235401/arduino-nano-hc-05-at-not-working
// https://www.gme.cz/data/attachments/dsh.772-148.2.pdf
// https://www.linkedin.com/pulse/add-arduino-ide-visual-studio-community-2017-new-project-homan-huang/
// https://www.itead.cc/wiki/Serial_Port_Bluetooth_Module_(Master/Slave)_:_HC-05#6._Set.2FCheck_module_name:
// http://denethor.wlu.ca/arduino/MLT-BT05-AT-commands-TRANSLATED.pdf

#define ROBOT_NAME "RandomBot"
#define BLUETOOTH_SPEED 38400 //This is the default baudrate that HC-05 uses
boolean BTconnected = false;
#define cmd 9 //EN pin
void setup() 
{
  
  // set input through EN pin
  pinMode(cmd, OUTPUT);
  digitalWrite(cmd, HIGH);
  //digitalWrite(TX, HIGH);

  //Serial turns on in 1 second. Be patient and wait.
  delay(1000);

  // wait until the HC-05 has made a connection
  //while (!BTconnected)
 // {
    //if (digitalRead(cmd) == HIGH) { BTconnected = true; };
  //}
  //pinMode(RX, INPUT); 
  //pinMode(TX, OUTPUT);  
  //digitalWrite(TX, HIGH);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("Starting config");
  // 9600 for http://denethor.wlu.ca/arduino/MLT-BT05-AT-commands-TRANSLATED.pdf
  mySerial.begin(9600); //  38400 
  delay(1000);
/*
  // Should respond with OK
  mySerial.print("AT\r\n");
  waitForResponse();

  // Should respond with its version
  mySerial.print("AT+VERSION\r\n");
  waitForResponse();

  // Set pin to 0000
  mySerial.print("AT+PIN000000\r\n");
  waitForResponse();

  // Set the name to ROBOT_NAME
  String rnc = String("AT+NAME") + String(ROBOT_NAME) + String("\r\n"); 
  mySerial.print(rnc);
  waitForResponse();

  // Set baudrate to 57600
  mySerial.print("AT+UART57600,0,0\r\n");
  waitForResponse();
*/
  Serial.println("Done!");
  /*
  pinMode(3, OUTPUT);
  pinMode(2, INPUT_PULLUP);
 //   pinMode(1, OUTPUT);
 // pinMode(0, INPUT_PULLUP);
    Serial.begin(9600);
    Serial.println("Enter AT commands:");
 
    // HC-06 default serial speed is 9600
    BTserial.begin(38400);  
    */
}
 
void loop()
{
    // Keep reading from HC-06 and send to Arduino Serial Monitor
   /* if (BTserial.available()) {  
        Serial.write(BTserial.read());
    }*/
    if (mySerial.available()) {  
        Serial.write(mySerial.read());
    }
 
    // Keep reading from Arduino Serial Monitor and send to HC-06
    if (Serial.available())
    {
      int s = Serial.read();
     //BTserial.write(s);
      mySerial.write(s);
     // Serial.write(s);
    }
}
