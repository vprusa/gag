  /*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
6e400003-b5a3-f393-e0a9-e50e24dcca9e
   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"
    6e400001-b5a3-f393-e0a9-e50e24dcca9e
6e400002-b5a3-f393-e0a9-e50e24dcca9e
6e400003-b5a3-f393-e0a9-e50e24dcca9e 
   The design of creating the BLE server is:
   1. Create a BLE Server
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

unsigned long timeNow, timeLastRead, timeLastWrite;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
//#define TEST_PERF 1
 

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
      timeNow = micros();
      
      if (rxValue.length() > 0) {
        Serial.print(timeNow);
        Serial.print(" ");
        unsigned long timeDiff = timeNow-timeLastRead;
        Serial.print(timeDiff);
        
        //Serial.println("*********");
        Serial.print(" Received: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();
        //Serial.println("*********");
        timeLastRead=timeNow;
      }
    }
};


void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  
  timeNow = micros();
  Serial.print("Current time is: ");
  Serial.println(timeNow);
}

uint8_t* string2char(String str){
    return reinterpret_cast<uint8_t*>(&str[0]);
}

void loop() {  
  if (deviceConnected) {
    if (Serial.available()){
      timeNow = micros();
      String str = Serial.readString();
      #ifdef TEST_PERF
      for(int i =0; i<10; i++){
      #endif
        timeNow = micros();
        Serial.print(timeNow);
        Serial.print(" ");
        unsigned long timeDiff = timeNow-timeLastWrite;
        Serial.print(timeDiff);
        Serial.print(" wrote: ");
        Serial.println(str);
      
        pTxCharacteristic->setValue(string2char(str), str.length());
        pTxCharacteristic->notify();
        timeLastWrite=timeNow;
      #ifdef TEST_PERF
      }
      #endif
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
	  // do stuff here on connecting
    Serial.println("connected");
    oldDeviceConnected = deviceConnected;
  }
}