

void setup(){
  // Initialize Serial Monitor
  Serial.begin(115200);//115200 38400 9600 57600
  Serial.println("ENTER AT Commands:");
  // Initialize Bluetooth Serial Port
  Serial2.begin(115200);
  delay(300);
  Serial.println("AT+BAUD\r\n");
  Serial2.print("AT+BAUD\r\n");
  delay(300);

 // while(!hc06.available()){}
 // Serial.println("hc06 avail");

//hc06.print("\r\n");
/*
hc06.print("AT+DEFAULT\r\n");
hc06.print("AT+RMAAD\r\n");
hc06.print("AT+ROLE1\r\n");
hc06.print("AT+BAND544A1608D741\r\n");
//hc06.print("\r\n");
*/  

}

//unsigned volatile int counter = 0;

void loop(){
  /*counter++;

  hc06.println(F("qwe"));
  delay(2000);
*/
  
  //Write data from HC06 to Serial Monitor
  if (Serial2.available()){
    Serial.write(Serial2.read());
  }
  
  //Write from Serial Monitor to HC06
  if (Serial.available()){
    //Serial.write(hc06.read());
    int c = Serial.read();
 //   if(c == '*'){
      //hc06.end();
     // hc06.begin(115200);
//    }else{
      Serial2.write(c);
    //  delay(1000);
 //   }
  }  
}
