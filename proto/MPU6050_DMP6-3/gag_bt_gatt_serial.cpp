// Copyright 2018 Vojtech Prusa
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


/*
 * Serial Bluetooth GATT Arduino
 *
 * */

BluetoothSerial::BluetoothSerial()
{
    //local_name = "ESP32"; //default bluetooth name
}

BluetoothSerial::~BluetoothSerial(void)
{
    //_stop_bt();
}

bool BluetoothSerial::begin(std::string localName, 
            BLEServerCallbacks* pServerCallbacks,
            BLECharacteristicCallbacks* pCharacteristcsCallbacks)
{
    // Create the BLE Device
    BLEDevice::init(localName);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(pServerCallbacks);

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                                    CHARACTERISTIC_UUID_TX,
                                    BLECharacteristic::PROPERTY_NOTIFY);
                    
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE);

    pRxCharacteristic->setCallbacks(pCharacteristcsCallbacks);
    // Start the service
    pService->start();
    // Start advertising
    pServer->getAdvertising()->start();
    //MASTER_SERIAL_NAME.println("Waiting a client connection to notify...");
    return true;
}

int BluetoothSerial::available(void)
{
    return 0;
}

int BluetoothSerial::peek(void)
{
    return -1;
}

/*
bool BluetoothSerial::hasClient(void)
{
    return true;
}
*/

int BluetoothSerial::read(void)
{
    return -1;
}

size_t BluetoothSerial::write(uint8_t c)
{
    return write(&c, 1);
}

//size_t BluetoothSerial::write(const uint8_t *buffer, size_t size)
size_t BluetoothSerial::write(uint8_t *buffer, size_t size)
{
    if (deviceConnected) {
        pTxCharacteristic->setValue(buffer, size);
        pTxCharacteristic->notify();
    }
    return 0;
}

void BluetoothSerial::flush()
{
    while(read() >= 0){}
}

void BluetoothSerial::end()
{
}

void BluetoothSerial::loop(){
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
