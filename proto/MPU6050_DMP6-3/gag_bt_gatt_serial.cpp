// Copyright 2019 Vojtech Prusa
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sdkconfig.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "WString.h"
#include "esp32-hal.h"

//#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)

#include "gag_bt_gatt_serial.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include <esp_log.h>

bool deviceConnected;

/*
 * Serial Bluetooth GATT Arduino
 *
 * */

BluetoothSerial::BluetoothSerial() {
    GAG_DEBUG_PRINTLN("BluetoothSerial");
    //local_name = "ESP32"; //default bluetooth name
}

BluetoothSerial::~BluetoothSerial(void) {
    GAG_DEBUG_PRINTLN("~BluetoothSerial");
    //_stop_bt();
}
/*
class ServerCallbacks2: public BLEServerCallbacks {            
    void onConnect(BLEServer* pServer) {
        GAG_DEBUG_PRINTLN("onConnect");
        //MASTER_SERIAL_NAME.deviceConnected = true;
        deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
        GAG_DEBUG_PRINTLN("onDisconnect");
        //MASTER_SERIAL_NAME.deviceConnected = false;
        deviceConnected = false;
    }
};

class CharCallbacks2: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
    GAG_DEBUG_PRINTLN("onWrite");
    std::string rxValue = pCharacteristic->getValue();
    
    if (rxValue.length() > 0) {
        GAG_DEBUG_PRINTLN(rxValue.c_str());
        for (int i = 0; i < rxValue.length(); i++)
            //SLAVE_SERIAL_NAME.print(rxValue[i]);
            Serial.print(rxValue[i]);
        }
    }
};*/

bool BluetoothSerial::begin(std::string localName, 
            ServerCallbacks* pServerCallbacksIn,
            CharCallbacks* pCharacteristcsCallbacksIn) {
    //#ifdef GAG_BT_GATT_SERIAL
        //Serial.begin(115200);
    //#endif
    pServerCallbacks = pServerCallbacksIn;
    pCharacteristcsCallbacks = pCharacteristcsCallbacksIn;

    // Create the BLE Device
    GAG_DEBUG_PRINTLN("begin");
    BLEDevice::init(localName);

    // Create the BLE Server
    GAG_DEBUG_PRINTLN("createServer");
    //Serial.println((int)pServerCallbacks);
    

    pServer = BLEDevice::createServer();
    delay(1);
    GAG_DEBUG_PRINTLN("pServer - setCallbacks");
    pServer->setCallbacks(pServerCallbacks);
    //pServer->setCallbacks(new BLEServerCallbacks());

    // Create the BLE Service
    GAG_DEBUG_PRINTLN("createService");
    BLEService *pService = pServer->createService(SERVICE_UUID);

    GAG_DEBUG_PRINTLN("pTxCharacteristic");
    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                                    CHARACTERISTIC_UUID_TX,
                                    BLECharacteristic::PROPERTY_NOTIFY);
                    
    pTxCharacteristic->addDescriptor(new BLE2902());

    GAG_DEBUG_PRINTLN("pRxCharacteristic");
    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE);
    
    GAG_DEBUG_PRINTLN("pRxCharacteristic - setCallbacks");
    pRxCharacteristic->setCallbacks(pCharacteristcsCallbacks);
    //pRxCharacteristic->setCallbacks(new CharCallbacks2());

    // Start the service
    GAG_DEBUG_PRINTLN("pService - start");
    pService->start();
    // Start advertising
    delay(1);
    GAG_DEBUG_PRINTLN("getAdvertising - start");
    pServer->getAdvertising()->start();
    //MASTER_SERIAL_NAME.println("Waiting a client connection to notify...");
    GAG_DEBUG_PRINTLN("begin - done");
    return true;
}

int BluetoothSerial::available(void) {
    GAG_DEBUG_PRINT("available - deviceConnected: ");
    GAG_DEBUG_PRINTLN(deviceConnected);
    return deviceConnected;
}

int BluetoothSerial::peek(void) {
    GAG_DEBUG_PRINTLN("peek");
    return -1;
}

/*
bool BluetoothSerial::hasClient(void) {
    return true;
}
*/

int BluetoothSerial::read(void) {
    GAG_DEBUG_PRINTLN("read");
    return -1;
}

size_t BluetoothSerial::write(uint8_t c) {
    //GAG_DEBUG_PRINTLN("write(uint8_t)");
    return write(&c, 1);
}

//size_t BluetoothSerial::write(const uint8_t *buffer, size_t size)
size_t BluetoothSerial::write(const uint8_t *buffer, size_t size) {
    //GAG_DEBUG_PRINTLN("write(const uint8_t*,size_t)");
    char * ch =   const_cast<char*>(reinterpret_cast<const char*>(buffer));
    //GAG_DEBUG_PRINT(ch);
    if (deviceConnected) {
        //GAG_DEBUG_PRINTLN("deviceConnected");
        pTxCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
        pTxCharacteristic->notify();
    }
    return 0;
}

size_t BluetoothSerial::print(const char* c) {
    GAG_DEBUG_PRINTLN("print(const char* c)");
    GAG_DEBUG_PRINT(c);
}

size_t BluetoothSerial::print(uint8_t c) {
    GAG_DEBUG_PRINTLN("print(uint8_t c)");
    GAG_DEBUG_PRINT(c);
}

size_t BluetoothSerial::println(uint8_t c) {
    GAG_DEBUG_PRINTLN("println(uint8_t c)");
    GAG_DEBUG_PRINTLN(c);
}

size_t BluetoothSerial::println(const uint8_t *buffer, size_t size) {
    GAG_DEBUG_PRINTLN("println(const uint8_t *buffer, size_t size)");
    char * ch =   const_cast<char*>(reinterpret_cast<const char*>(buffer));
    GAG_DEBUG_PRINTLN(ch);
    //GAG_DEBUG_PRINTLNF(buffer, size);
    //size_t Print::write(const uint8_t *buffer, size_t size)
}

size_t BluetoothSerial::println(unsigned char b, int base) {
    GAG_DEBUG_PRINTLN("println(unsigned char b, int base)");
    GAG_DEBUG_PRINTLNF(b, base);
    // size_t Print::println(unsigned char b, int base)
}

/*
size_t BluetoothSerial::print(const char* buffer) {
    GAG_DEBUG_PRINTLN("print(const char*");
    GAG_DEBUG_PRINT(buffer);
}
*/
size_t BluetoothSerial::println(const char* buffer) {
    GAG_DEBUG_PRINTLN("println(const char*");
    GAG_DEBUG_PRINTLN(buffer);
}

size_t BluetoothSerial::print(const __FlashStringHelper *ifsh) {
    GAG_DEBUG_PRINTLN("print(const __FlashStringHelper *ifsh)");
    GAG_DEBUG_PRINTLN(ifsh);
}

size_t BluetoothSerial::println(const __FlashStringHelper *ifsh) {
    GAG_DEBUG_PRINTLN("println(const __FlashStringHelper *ifsh)");
    GAG_DEBUG_PRINTLN(ifsh);
    //size_t n = print(ifsh);
    //n += println();
    //return n;
}

void BluetoothSerial::flush() {
    GAG_DEBUG_PRINTLN("flush");
    while(read() >= 0){}
}

void BluetoothSerial::end() {
    GAG_DEBUG_PRINTLN("end");
    deviceConnected = false;
}

void BluetoothSerial::loop() {
    //GAG_DEBUG_PRINTLN("loop");
      // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        //MASTER_SERIAL_NAME.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        //MASTER_SERIAL_NAME.println("connected");
        oldDeviceConnected = deviceConnected;
    }
}

/*
esp_err_t BluetoothSerial::register_callback(esp_spp_cb_t * callback)
{
    //custom_spp_callback = callback;
    return ESP_OK;
}
*/


// MASTER_BT_SERIAL_NAME
