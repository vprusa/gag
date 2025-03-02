// I2Cdev library collection - MPU9150 I2C device class, 9-axis MotionApps 4.1 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 6/18/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _MPU9150_9AXIS_MOTIONAPPS41_H_
#define _MPU9150_9AXIS_MOTIONAPPS41_H_

#include "I2Cdev.h"
#include "helper_3dmath.h"

// MotionApps 4.1 DMP implementation, built using the MPU-9150 "MotionFit" board
#define MPU9150_INCLUDE_DMP_MOTIONAPPS41

#include "MPU9150.h"

#ifndef ESP32_RIGHT
#include <avr/pgmspace.h>
#endif

    // Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
    #ifdef __PGMSPACE_H_
        #define __PGMSPACE_H_ 1
        #include <inttypes.h>

        #define PROGMEM
        #define PGM_P  const char *
        #define PSTR(str) (str)
        #define F(x) x

        typedef void prog_void;
        typedef char prog_char;
        typedef unsigned char prog_uchar;
        //vprusa
        //typedef int8_t prog_int8_t;
        typedef uint8_t prog_uint8_t;
        typedef int16_t prog_int16_t;
        typedef uint16_t prog_uint16_t;
        
        //vprusa
        //typedef int32_t prog_int32_t;
        //typedef uint32_t prog_uint32_t;
        
        #define strcpy_P(dest, src) strcpy((dest), (src))
        #define strcat_P(dest, src) strcat((dest), (src))
        #define strcmp_P(a, b) strcmp((a), (b))
        
        #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
        #define pgm_read_word(addr) (*(const unsigned short *)(addr))
        #define pgm_read_dword(addr) (*(const unsigned long *)(addr))
        #define pgm_read_float(addr) (*(const float *)(addr))
        
        #define pgm_read_byte_near(addr) pgm_read_byte(addr)
        #define pgm_read_word_near(addr) pgm_read_word(addr)
        #define pgm_read_dword_near(addr) pgm_read_dword(addr)
        #define pgm_read_float_near(addr) pgm_read_float(addr)
        #define pgm_read_byte_far(addr) pgm_read_byte(addr)
        #define pgm_read_word_far(addr) pgm_read_word(addr)
        #define pgm_read_dword_far(addr) pgm_read_dword(addr)
        #define pgm_read_float_far(addr) pgm_read_float(addr)
    #endif

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).

//#define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#define MPU9150_DMP_CODE_SIZE       1962    // dmpMemory[]
#define MPU9150_DMP_CONFIG_SIZE     232     // dmpConfig2[]
#define MPU9150_DMP_UPDATES_SIZE    140     // dmpUpdates2[]


/* ================================================================================================ *
 | Default MotionApps v4.1 48-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][MAG X ][MAG Y ][MAG Z ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ] |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  |
 * ================================================================================================ */


/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

uint8_t MPU9150::dmpInitialize() {
    // reset device
    DEBUG_PRINTLN(F("\n\nResetting MPU9150..."));
    reset();
    delay(30); // wait after reset

    // disable sleep mode
    DEBUG_PRINTLN(F("Disabling sleep mode..."));
    setSleepEnabled(false);

    // get MPU product ID
    DEBUG_PRINTLN(F("Getting product ID..."));
    uint8_t productID = 0; //getProductID();
    DEBUG_PRINT(F("Product ID = "));
    DEBUG_PRINT(productID);

    // get MPU hardware revision
    DEBUG_PRINTLN(F("Selecting user bank 16..."));
    setMemoryBank(0x10, true, true);
    DEBUG_PRINTLN(F("Selecting memory byte 6..."));
    setMemoryStartAddress(0x06);
    DEBUG_PRINTLN(F("Checking hardware revision..."));
    uint8_t hwRevision = readMemoryByte();
    DEBUG_PRINT(F("Revision @ user[16][6] = "));
    DEBUG_PRINTLNF(hwRevision, HEX);
    DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
    setMemoryBank(0, false, false);

    // check OTP bank valid
    DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
    uint8_t otpValid = getOTPBankValid();
    DEBUG_PRINT(F("OTP bank is "));
    DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    DEBUG_PRINTLN(F("Reading gyro offset values..."));
    int8_t xgOffset = getXGyroOffset();
    int8_t ygOffset = getYGyroOffset();
    int8_t zgOffset = getZGyroOffset();
    DEBUG_PRINT(F("X gyro offset = "));
    DEBUG_PRINTLN(xgOffset);
    DEBUG_PRINT(F("Y gyro offset = "));
    DEBUG_PRINTLN(ygOffset);
    DEBUG_PRINT(F("Z gyro offset = "));
    DEBUG_PRINTLN(zgOffset);
    
    I2Cdev::readByte(devAddr, MPU9150_RA_USER_CTRL, buffer); // ?
    
    DEBUG_PRINTLN(F("Enabling interrupt latch, clear on any read, AUX bypass enabled"));
    I2Cdev::writeByte(devAddr, MPU9150_RA_INT_PIN_CFG, 0x32);

    // enable MPU AUX I2C bypass mode
    //DEBUG_PRINTLN(F("Enabling AUX I2C bypass mode..."));
    //setI2CBypassEnabled(true);

    DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
    //mag -> setMode(0);
    I2Cdev::writeByte(0x0E, 0x0A, 0x00);

    DEBUG_PRINTLN(F("Setting magnetometer mode to fuse access..."));
    //mag -> setMode(0x0F);
    I2Cdev::writeByte(0x0E, 0x0A, 0x0F);

    DEBUG_PRINTLN(F("Reading mag magnetometer factory calibration..."));
    int8_t asax, asay, asaz;
    //mag -> getAdjustment(&asax, &asay, &asaz);
    I2Cdev::readBytes(0x0E, 0x10, 3, buffer);
    asax = (int8_t)buffer[0];
    asay = (int8_t)buffer[1];
    asaz = (int8_t)buffer[2];
    DEBUG_PRINT(F("Adjustment X/Y/Z = "));
    DEBUG_PRINT(asax);
    DEBUG_PRINT(F(" / "));
    DEBUG_PRINT(asay);
    DEBUG_PRINT(F(" / "));
    DEBUG_PRINTLN(asaz);

    DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
    //mag -> setMode(0);
    I2Cdev::writeByte(0x0E, 0x0A, 0x00);

    DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
    setDMPEnabled(false);

    DEBUG_PRINTLN(F("Setting up internal 48-byte (default) DMP packet buffer..."));
    dmpPacketSize = 48;
    /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
        return 3; // TODO: proper error code for no memory
    }*/

    DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
    resetFIFO();
    getIntStatus();

    return 0; // success
}

bool MPU9150::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

uint8_t MPU9150::dmpGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[34] << 24) + (packet[35] << 16) + (packet[36] << 8) + packet[37]);
    data[1] = ((packet[38] << 24) + (packet[39] << 16) + (packet[40] << 8) + packet[41]);
    data[2] = ((packet[42] << 24) + (packet[43] << 16) + (packet[44] << 8) + packet[45]);
    return 0;
}
uint8_t MPU9150::dmpGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[34] << 8) + packet[35];
    data[1] = (packet[38] << 8) + packet[39];
    data[2] = (packet[42] << 8) + packet[43];
    return 0;
}
uint8_t MPU9150::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[34] << 8) + packet[35];
    v -> y = (packet[38] << 8) + packet[39];
    v -> z = (packet[42] << 8) + packet[43];
    return 0;
}
uint8_t MPU9150::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 24) + (packet[1] << 16) + (packet[2] << 8) + packet[3]);
    data[1] = ((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]);
    data[2] = ((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]);
    data[3] = ((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]);
    return 0;
}
uint8_t MPU9150::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) + packet[1]);
    data[1] = ((packet[4] << 8) + packet[5]);
    data[2] = ((packet[8] << 8) + packet[9]);
    data[3] = ((packet[12] << 8) + packet[13]);
    return 0;
}
uint8_t MPU9150::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
// uint8_t MPU9150::dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t MPU9150::dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t MPU9150::dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[16] << 24) + (packet[17] << 16) + (packet[18] << 8) + packet[19]);
    data[1] = ((packet[20] << 24) + (packet[21] << 16) + (packet[22] << 8) + packet[23]);
    data[2] = ((packet[24] << 24) + (packet[25] << 16) + (packet[26] << 8) + packet[27]);
    return 0;
}
uint8_t MPU9150::dmpGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) + packet[17];
    data[1] = (packet[20] << 8) + packet[21];
    data[2] = (packet[24] << 8) + packet[25];
    return 0;
}
uint8_t MPU9150::dmpGetMag(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[28] << 8) + packet[29];
    data[1] = (packet[30] << 8) + packet[31];
    data[2] = (packet[32] << 8) + packet[33];
    return 0;
}
// uint8_t MPU9150::dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MPU9150::dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t MPU9150::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v -> x = vRaw -> x - gravity -> x*4096;
    v -> y = vRaw -> y - gravity -> y*4096;
    v -> z = vRaw -> z - gravity -> z*4096;
    return 0;
}
// uint8_t MPU9150::dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t MPU9150::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}
uint8_t MPU9150::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}
uint8_t MPU9150::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}
uint8_t MPU9150::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

uint8_t MPU9150::dmpProcessFIFOPacket(const unsigned char *dmpData) {
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t MPU9150::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;
        
        // increment external process count variable, if supplied
        if (processed != 0) *processed++;
    }
    return 0;
}

uint16_t MPU9150::dmpGetFIFOPacketSize() {
    return dmpPacketSize;
}

#endif /* _MPU9150_9AXIS_MOTIONAPPS41_H_ */
