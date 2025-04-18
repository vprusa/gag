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

#include "definitions.h"

#ifdef MASTER_HAND


#include "sdkconfig.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

/**
 * Serial Bluetooth GATT Arduino
 *
 */
BluetoothSerial::BluetoothSerial() {
    GAG_BT_DEBUG_PRINTLN("BluetoothSerial");
    //local_name = "ESP32"; //default bluetooth name
    // serialBuffer = new char[BTSerialBufferSize][];
    for(char i = 0; i++; i < BT_SERIAL_BUFFER_SIZE){
        for(char ii = 0; ii++; ii < CMD_PACKET_LENGTH){
            serialBuffer[i][ii] = -1;
        }
    }
    serialBufferWriteIndex = 0;
    serialBufferReadIndex = 0;
}

BluetoothSerial::~BluetoothSerial(void) {
    GAG_BT_DEBUG_PRINTLN("~BluetoothSerial");
}

bool BluetoothSerial::begin(std::string localName, 
            ServerCallbacks* pServerCallbacksIn,
            CharCallbacks* pCharacteristcsCallbacksIn) {
   
    pServerCallbacks = pServerCallbacksIn;
    pCharacteristcsCallbacks = pCharacteristcsCallbacksIn;

    // Create the BLE Device
    GAG_BT_DEBUG_PRINTLN("begin");
    BLEDevice::init(localName);

    // Create the BLE Server
    GAG_BT_DEBUG_PRINTLN("createServer");
   
    pServer = BLEDevice::createServer();
    
    //pServer->updateConnParams(esp_ble_conn_update_params_t *params);
    //esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    // pServer->getAdvertising()->setMinPreferred(0x06);  // Minimum preferred connection interval
    // pServer->getAdvertising()->setMinPreferred(0x12);  // Maximum preferred connection interval

    esp_ble_conn_update_params_t conn_params = {0};

    conn_params.min_int = 0x10;  // 20ms (recommended: 0x10 to 0x30)
    conn_params.max_int = 0x30;  // 60ms (recommended: 0x10 to 0x30)
    conn_params.latency = 0;     // No slave latency
    conn_params.timeout = 600;   // 6 seconds supervision timeout

    uint16_t conn_id = pServer->getConnId(); // Get connection ID if available
    if (conn_id) {
        esp_ble_gap_update_conn_params(&conn_params);
        GAG_DEBUG_PRINTLN("Connection parameters updated");
    }


    delay(1);
    GAG_BT_DEBUG_PRINTLN("pServer - setCallbacks");
    pServer->setCallbacks(pServerCallbacks);
    

    // Create the BLE Service
    GAG_BT_DEBUG_PRINTLN("createService");
    BLEService *pService = pServer->createService(SERVICE_UUID);

    GAG_BT_DEBUG_PRINTLN("pTxCharacteristic");
    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                                    CHARACTERISTIC_UUID_TX,
                                    BLECharacteristic::PROPERTY_NOTIFY);
                    
    ble2902 = new BLE2902();
    //ble2902->setIndications(true);
    //ble2902->setNotifications(true);
    //pTxCharacteristic->setWriteNoResponseProperty(true);
    //pTxCharacteristic->setNotifyProperty(true);
    //pTxCharacteristic->setIndicateProperty(true);
    //pTxCharacteristic->setAccessPermissions();
    pTxCharacteristic->addDescriptor(ble2902);

    GAG_BT_DEBUG_PRINTLN("pRxCharacteristic");
    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE);
    GAG_BT_DEBUG_PRINTLN("pRxCharacteristic - setCallbacks");
    pRxCharacteristic->setCallbacks(pCharacteristcsCallbacks);

    // Start the service
    GAG_BT_DEBUG_PRINTLN("pService - start");
    pService->start();
    // Start advertising
    delay(1);
    GAG_BT_DEBUG_PRINTLN("getAdvertising - start");
    pServer->getAdvertising()->start();
    GAG_BT_DEBUG_PRINTLN("begin - done");
    return true;
}

int BluetoothSerial::available(void) {
    GAG_BT_DEBUG_PRINT("available - deviceConnected: ");
    GAG_BT_DEBUG_PRINTLN(deviceConnected);
    return deviceConnected;
}

int BluetoothSerial::peek(void) {
    GAG_BT_DEBUG_PRINTLN("peek");
    return -1;
}

int BluetoothSerial::read(void) {
    //GAG_BT_DEBUG_PRINTLN("read");
    int8_t c = serialBuffer[serialBufferReadIndex][serialBufferReadCharIndex];
    if(c!=-1){
        // GAG_BT_DEBUG_PRINT((char)c);
        // GAG_BT_DEBUG_PRINT(" ");
        // GAG_BT_DEBUG_PRINT(c);
        // GAG_BT_DEBUG_PRINT(" ");
        // GAG_BT_DEBUG_PRINT(serialBufferReadIndex);
        // GAG_BT_DEBUG_PRINT(" ");
        // GAG_BT_DEBUG_PRINTLN(serialBufferReadCharIndex);
        
        if(++serialBufferReadCharIndex>=CMD_PACKET_LENGTH){
            for(char i =0; i<CMD_PACKET_LENGTH; i++) {
                serialBuffer[serialBufferReadIndex][i]=-1;
            }
            serialBufferReadCharIndex = 0;
            if(++serialBufferReadIndex>=BT_SERIAL_BUFFER_SIZE){serialBufferReadIndex=0;}
        }
    }
    return c;
}

/*
char* BluetoothSerial::read(uint8_t len) {
    //GAG_BT_DEBUG_PRINTLN("read");
    if(len == CMD_PACKET_LENGTH){
        char * ch = serialBuffer[serialBufferReadIndex];
        if(serialBufferReadIndex++>BT_SERIAL_BUFFER_SIZE){serialBufferReadIndex = 0;}
        return ch;
    }
    return c;
}
*/

size_t BluetoothSerial::write(uint8_t c) {
    return write(&c, 1);
}

size_t BluetoothSerial::write(const uint8_t *buffer, size_t size) {
    char * ch =   const_cast<char*>(reinterpret_cast<const char*>(buffer));
    // GAG_BT_DEBUG_PRINT("BluetoothSerial:write(const uint8_t *buffer, size_t size) ");
    // GAG_BT_DEBUG_PRINTLN(size);
    if (deviceConnected) {
        // GAG_BT_DEBUG_PRINTLN(ch);

        //pTxCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
        //pTxCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
        //pTxCharacteristic->setValue(const_cast<char*>(reinterpret_cast<const char*>(buffer)), size);
        //pTxCharacteristic->indicate();
        //pTxCharacteristic->setValue(dynamic_cast<uint8_t*>(buffer), size);
        //pTxCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
        //pTxCharacteristic->notify(true);
        //pTxCharacteristic->getDescriptorByUUID(CHARACTERISTIC_UUID_TX)->setValue(value);
        // pTxCharacteristic->getDescriptorByUUID(CHARACTERISTIC_UUID_TX)->setValue(const_cast<uint8_t*>(buffer), size);

        // BLEService* s = (BLEService*) 
        // pTxCharacteristic->setWriteNoResponseProperty(true);

        //pTxCharacteristic->setNotifyProperty(true);
        //pTxCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
        // pTxCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t) 0x2902))->setValue(const_cast<uint8_t*>(buffer), size);
        pTxCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
        // std::string value((char*) buffer, size);
        // pTxCharacteristic->setValue(value);
        //ble2902->setValue(value);
        
        //->getCharacteristic(CHARACTERISTIC_UUID_TX)->setValue(const_cast<uint8_t*>(buffer), size);
        //pTxCharacteristic->setValue((char*) buffer, size);
        // pTxCharacteristic->notify(true);
        //pTxCharacteristic->notify(false);
        pTxCharacteristic->notify();
        // pTxCharacteristic->notify(true);
        //delay(1);
    }
    return 0;
}

size_t BluetoothSerial::print(const char* c) {
    GAG_BT_DEBUG_PRINTLN("print(const char* c)");
    GAG_BT_DEBUG_PRINT(c);
}

size_t BluetoothSerial::print(uint8_t c) {
    GAG_BT_DEBUG_PRINTLN("print(uint8_t c)");
    GAG_BT_DEBUG_PRINT(c);
}

size_t BluetoothSerial::println(uint8_t c) {
    GAG_BT_DEBUG_PRINTLN("println(uint8_t c)");
    GAG_BT_DEBUG_PRINTLN(c);
}

size_t BluetoothSerial::println(const uint8_t *buffer, size_t size) {
    GAG_BT_DEBUG_PRINTLN("println(const uint8_t *buffer, size_t size)");
    char * ch =   const_cast<char*>(reinterpret_cast<const char*>(buffer));
    GAG_BT_DEBUG_PRINTLN(ch);
}

size_t BluetoothSerial::println(unsigned char b, int base) {
    GAG_BT_DEBUG_PRINTLN("println(unsigned char b, int base)");
    GAG_BT_DEBUG_PRINTLNF(b, base);
}

size_t BluetoothSerial::println(const char* buffer) {
    //GAG_BT_DEBUG_PRINTLN("println(const char*");
    GAG_BT_DEBUG_PRINTLN(buffer);
}

size_t BluetoothSerial::print(const __FlashStringHelper *ifsh) {
    GAG_BT_DEBUG_PRINTLN("print(const __FlashStringHelper *ifsh)");
    GAG_BT_DEBUG_PRINTLN(ifsh);
}

size_t BluetoothSerial::println(const __FlashStringHelper *ifsh) {
    GAG_BT_DEBUG_PRINTLN("println(const __FlashStringHelper *ifsh)");
    GAG_BT_DEBUG_PRINTLN(ifsh);
    //size_t n = print(ifsh);
    //n += println();
    //return n;
}

void BluetoothSerial::flush() {
    GAG_BT_DEBUG_PRINTLN("flush");
    while(read() >= 0){}
}

void BluetoothSerial::end() {
    GAG_BT_DEBUG_PRINTLN("end");
    deviceConnected = false;
}

void BluetoothSerial::loop() {
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

#endif