// Copyright 2018 Vojtech Prusa
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


#include "sdkconfig.h"

//#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)

#include "Arduino.h"
#include "Stream.h"
#include <esp_spp_api.h>


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
//#define TEST_PERF 

class BluetoothSerial: public Stream
{
    public:

        BluetoothSerial(void);
        ~BluetoothSerial(void);

        // bool begin(String localName=String());
        bool begin(std::string deviceName,
            BLEServerCallbacks* pServerCallbacks,
            BLECharacteristicCallbacks* pCharacteristcsCallbacks);
        int available(void);
        int peek(void);
        //bool hasClient(void);
        int read(void);
        size_t write(uint8_t c);
        size_t write(uint8_t *buffer, size_t size);
        //size_t write(const uint8_t *buffer, size_t size);
        void flush();
        void end(void);
        //esp_err_t register_callback(esp_spp_cb_t * callback);
        void loop();

        BLEServer *pServer;
        BLECharacteristic * pTxCharacteristic;
        bool deviceConnected;
        bool oldDeviceConnected;

    private:
        //String local_name;

};


#endif