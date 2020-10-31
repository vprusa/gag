 
#define RXD2 16
#define TXD2 17

void setup() {
  //Serial.begin(9600);
  //Serial.println("Starting Serial");
  
  //Serial1.begin(9600);
  //Serial1.println("Starting Serial1");

  //Serial2.begin(9600);
  //Serial2.println("Starting Serial2");
   // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  /*Serial.begin(9600);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
  Serial.println("Serial2 Txd is on pin: "+String(TXD2));
  Serial.println("Serial2 Rxd is on pin: "+String(RXD2));
  Serial.println("Try Serial2: AT\\r\\n");
  Serial2.print("AT\r\n");*/
  Serial.begin(9600);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
  Serial.println("Serial2 Txd is on pin: "+String(TXD2));
  Serial.println("Serial2 Rxd is on pin: "+String(RXD2));
  Serial.println("Try Serial2: AT\\r\\n");
  Serial2.print("AT\r\n");

}

void loop() {
  /*if (Serial.available()) {
    Serial.write(Serial.read());
  }
  if (Serial.available()) {
    Serial.write(Serial.read());
  }*/

/*  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }*/

 /* if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }*/

  /*if (Serial2.available()) {
    Serial.write(Serial2.read());
  }*/
  
  /*if (Serial1.available()) {
    Serial.write(Serial1.read());
  }*/
  /*
  if (Serial.available()) {
    int c = Serial.read();
    //Serial1.write(c);
    Serial2.write(c);
  }*/

/*
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
*/
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
}
