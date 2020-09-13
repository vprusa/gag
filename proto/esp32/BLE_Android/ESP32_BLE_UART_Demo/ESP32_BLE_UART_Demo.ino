/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Serverv
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"



BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int txValue, tyValue, tbValue;

// 32 , 32, 34
const int readPinB = 32;
const int readPinX = 34;
const int readPinY = 35;
const int LED = 2; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

//std::string rxValue; // Could also make this a global var to access it in loop()

// button hold means pressed t>1s
// button press means click
// or TODO rework and 1B value can be time pressed in [ms*10] or smth...
// Flag 1B = B(button), counter [1B], buddon1 pressed [1B], buddon1 hold [1B], button 2 pressed [1B], button 2 hold [1B]
// Flag [1B] = D(data), counter [1B], X [2B], Y [2B], Z [1B]
// #define PACKET_LENGTH 9
// uint8_t packet[PACKET_LENGTH] = {'D', 0,  0, 0,  0, 0,  0, '\r', '\n'};
// Flag [1B] = D(data), counter [1B], X [1B], Y [1B], Z [1B], Button1 [1B], Button2 [1B]
#define PACKET_LENGTH 9
uint8_t packet[PACKET_LENGTH] = {'D', 0,  0,  0,  0,  0,  0, '\r', '\n'};

void printPacket(){
  Serial.print("Packet: ");
  // Serial.println(dynamic_cast<char*>(packet));
  Serial.print(" X: ");
  Serial.print(static_cast<int>(packet[2]));
  Serial.print(" Y: ");
  Serial.print(static_cast<int>(packet[3]));
  Serial.print(" Z: ");
  Serial.print(static_cast<int>(packet[4]));
  Serial.print(" ");

  Serial.write(packet, PACKET_LENGTH);
  Serial.print(" ");

}

void setPacket() {
  static uint8_t counter = 0;
  // packet[] = ;
  packet[1] = ++counter;
  packet[2] = txValue / 4;
  packet[3] = tyValue / 4;
  packet[4] = tbValue; // 0 = pressed, not pressed is 4-30 (TODO solve flicking to 0)
  // packet[5] = b1;
  // packet[6] = b2;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();

        // Do stuff based on the command received from the app
        if (rxValue.find("A") != -1) { 
          Serial.print("Turning ON!");
          digitalWrite(LED, HIGH);
        }
        else if (rxValue.find("B") != -1) {
          Serial.print("Turning OFF!");
          digitalWrite(LED, LOW);
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};


void setup() {
  
  Serial.begin(115200);

  pinMode(LED, OUTPUT);


  // Create the BLE Device
  BLEDevice::init("ESP32 UART Test"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );


  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();


  Serial.println("Waiting a client connection to notify...");
  pinMode(readPinX,   INPUT);
  pinMode(readPinY,   INPUT);
  pinMode(readPinB,   INPUT);

}

void loop() {
  txValue = analogRead(readPinX); // This could be an actual sensor reading!
  tyValue = analogRead(readPinY); // This could be an actual sensor reading!
  tbValue = analogRead(readPinB); // This could be an actual sensor reading!

  
  // Let's convert the value to a char array:
  char txString[8]; // make sure this is big enuffz
  char tyString[8]; // make sure this is big enuffz
//    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
  itoa(txValue,txString,10);
  itoa(tyValue,tyString,10);

  char combAr[17];

  strcpy(combAr, txString);
  strcat(combAr, ";");
  strcat(combAr, tyString);

//    pCharacteristic->setValue(&txValue, 1); // To send the integer value
//    pCharacteristic->setValue("Hello!"); // Sending a test message
  Serial.print("*** Sent Value: ");
  Serial.print(txValue);
  Serial.print(" ");
  Serial.print(txString);
  Serial.print(" ");
  Serial.print(tyValue);
  Serial.print(" ");
  Serial.print(tyString);
  Serial.print(" ");
  Serial.print(tbValue);
  Serial.print(" ");
  Serial.print(combAr);
//    Serial.print(" ");
//    Serial.print(tbString);
  Serial.println(" ***");

  setPacket();
  printPacket();

  if (deviceConnected) {
    // Fabricate some arbitrary junk for now...

    // pCharacteristic->setValue(txString);
    // pCharacteristic->setValue(combAr);
    // pCharacteristic->setValue(dynamic_cast<ch>(packet));
    pCharacteristic->setValue(const_cast<uint8_t*>(packet), PACKET_LENGTH);
    pCharacteristic->notify(); // Send the value to the app!
    
    // You can add the rxValue checks down here instead
    // if you set "rxValue" as a global var at the top!
    // Note you will have to delete "std::string" declaration
    // of "rxValue" in the callback function.
//    if (rxValue.find("A") != -1) { 
//      Serial.println("Turning ON!");
//      digitalWrite(LED, HIGH);
//    }
//    else if (rxValue.find("B") != -1) {
//      Serial.println("Turning OFF!");
//      digitalWrite(LED, LOW);
//    }
  }
  delay(500);
}
