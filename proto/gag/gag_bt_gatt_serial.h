// Copyright 2019 Vojtech Prusa
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _GAG_BT_GATT_SERIAL_H_
#define _GAG_BT_GATT_SERIAL_H_

#define GAG_BT_GATT_SERIAL
#ifdef GAG_BT_GATT_SERIAL
    #define GAG_BT_DEBUG_PRINT(x) Serial.print(x)
    #define GAG_BT_DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define GAG_BT_DEBUG_PRINTLN(x) Serial.println(x)
    #define GAG_BT_DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define GAG_BT_DEBUG_PRINT(x)
    #define GAG_BT_DEBUG_PRINTF(x, y)
    #define GAG_BT_DEBUG_PRINTLN(x)
    #define GAG_BT_DEBUG_PRINTLNF(x, y)
#endif

#include "sdkconfig.h"

//#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)

#include "Arduino.h"
#include "Stream.h"
#include <esp_spp_api.h>


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "definitions.h"


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
//#define TEST_PERF 
#define BT_SERIAL_BUFFER_SIZE 2

// MASTER_BT_SERIAL_NAME


class ServerCallbacks: public BLEServerCallbacks {
    public:            

        ServerCallbacks(void);
        ~ServerCallbacks(void);
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
};

class CharCallbacks: public BLECharacteristicCallbacks {
    public:
        CharCallbacks(void);
        ~CharCallbacks(void);
        void onWrite(BLECharacteristic *pCharacteristic);
};

// TODO implement properly.. or override sending just 1 char ...?
class BluetoothSerial //: public Stream
{
    public:

        BluetoothSerial(void);
        ~BluetoothSerial(void);

        // bool begin(String localName=String());
        bool begin(std::string deviceName,
            ServerCallbacks*,
            CharCallbacks*);
        int available(void);
        int peek(void);
        //bool hasClient(void);
        int read(void);
        size_t write(uint8_t c);
        size_t write(const uint8_t *buffer, size_t size);
        
        size_t print(const char* c);
        size_t print(uint8_t c);
        size_t println(const uint8_t *buffer, size_t size);
        size_t println(unsigned char b, int base);
        //size_t print(const char* buffer);
        size_t println(const char* buffer); 
        size_t println(uint8_t);
        
        size_t print(const __FlashStringHelper *ifsh);
        size_t println(const __FlashStringHelper *ifsh);
        
        //size_t write(const uint8_t *buffer, size_t size);
        void flush();
        void end(void);
        //esp_err_t register_callback(esp_spp_cb_t * callback);
        void loop();

        // TODO privateize and then mine
        BLEServer *pServer;
        BLECharacteristic * pTxCharacteristic;
        //bool deviceConnected;
        bool oldDeviceConnected;

        ServerCallbacks * pServerCallbacks;      
        CharCallbacks * pCharacteristcsCallbacks;


        int8_t serialBuffer[BT_SERIAL_BUFFER_SIZE][CMD_PACKET_LENGTH];
        uint8_t serialBufferWriteIndex;
        uint8_t serialBufferReadIndex;
        uint8_t serialBufferReadCharIndex;
        BLE2902 * ble2902;

    private:
        //String local_name;

};


#endif