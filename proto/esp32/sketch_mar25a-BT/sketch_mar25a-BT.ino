

/*Program to control LED (ON/OFF) from ESP32 using Serial Bluetooth
 * Thanks to Neil Kolbans for his efoorts in adding the support to Arduino IDE
 * Turotial on: www.circuitdigest.com 
 */

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino

BluetoothSerial ESP_BT; //Object for Bluetooth

int incoming;
int LED_BUILTIN = 2;

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200); //Start Serial monitor in 9600
  ESP_BT.begin("ESP32_LED_Control"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");

  pinMode (LED_BUILTIN, OUTPUT);//Specify that LED pin is output
}

void loop() {
  //Serial.println("A\n");
  if (ESP_BT.available()) //Check if we receive anything from Bluetooth
  {
    incoming = ESP_BT.read(); //Read what we recevive 
    Serial.println("Received:"); Serial.println(incoming);

    if (incoming == 49)
        {
        digitalWrite(LED_BUILTIN, HIGH);
        ESP_BT.println("LED turned ON");
        }
        
    if (incoming == 48)
        {
        digitalWrite(LED_BUILTIN, LOW);
        ESP_BT.println("LED turned OFF");
        }     
  }
  delay(20);
}

