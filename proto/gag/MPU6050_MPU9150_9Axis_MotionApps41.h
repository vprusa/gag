// I2Cdev library collection - MPU6050_MPU9150 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
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

#ifndef _MPU6050_MPU9150_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_MPU9150_6AXIS_MOTIONAPPS20_H_

#include "I2Cdev.h"
//#include "esp32-hal-i2c.h"

#include "helper_3dmath.h"

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_MPU9150_INCLUDE_DMP_MOTIONAPPS20

#include "MPU6050_MPU9150.h"

// Tom Carpenter's conditional PROGMEM code
// http://forum.arduino.cc/index.php?topic=129407.0

#ifndef ESP32_RIGHT 
    #define __PGMSPACE_H_ 1
    #include <inttypes.h>

   // #define PROGMEM 
    // wtf
    //#define PGM_P  const char *
    //#define PSTR(str) (str)
    //#define F(x) x

    typedef void prog_void;
    typedef char prog_char;
    typedef unsigned char prog_uchar;
    #ifdef SLAVE_HAND
    //vprusa
    typedef int8_t prog_int8_t;
    typedef uint8_t prog_uint8_t;
    typedef int16_t prog_int16_t;
    typedef uint16_t prog_uint16_t;
    
    //vprusa
    typedef int32_t prog_int32_t;
    typedef uint32_t prog_uint32_t;
    #endif
    /*
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
    */
#endif

//#define __AVR__
#ifdef __AVR__
    #include <avr/pgmspace.h>
#else
    // Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
    #ifndef __PGMSPACE_H_
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
#endif

/* Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).

#define DEBUG
#ifdef DEBUG
    #define MPU_DEBUG_PRINT(x) Serial.print(x)
    #define MPU_DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define MPU_DEBUG_PRINTLN(x) Serial.println(x)
    #define MPU_DEBUG_PRINTLNF(x, y) Serial.println(x, y)
    #define DEBUG_WRITE(x) Serial.write(x)
    #define DEBUG_WRITE_LEN(x,y) Serial.write(x,y)
#else
    #define MPU_DEBUG_PRINT(x)
    #define MPU_DEBUG_PRINTF(x, y)
    #define MPU_DEBUG_PRINTLN(x)
    #define MPU_DEBUG_PRINTLNF(x, y)
#endif

#define MPU9150_DEBUG
#ifdef MPU9150_DEBUG
    #define MPU9150_MPU_DEBUG_PRINT(x) Serial.print(x)
    #define MPU9150_MPU_DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define MPU9150_MPU_DEBUG_PRINTLN(x) Serial.println(x)
    #define MPU9150_MPU_DEBUG_PRINTLNF(x, y) Serial.println(x, y)
    #define MPU9150_DEBUG_WRITE(x) Serial.write(x)
    #define MPU9150_DEBUG_WRITE_LEN(x,y) Serial.write(x,y)
#else
    #define MPU9150_MPU_DEBUG_PRINT(x)
    #define MPU9150_MPU_DEBUG_PRINTF(x, y)
    #define MPU9150_MPU_DEBUG_PRINTLN(x)
    #define MPU9150_MPU_DEBUG_PRINTLNF(x, y)
#endif

#define MPU6050_MPU9150_DMP_CODE_SIZE 512

//#define MPU9150_DMP_CODE_SIZE       1962    // dmpMemory[]
#define MPU9150_DMP_CODE_SIZE       1450 //1450    // dmpMemory[]
#define MPU9150_DMP_CONFIG_SIZE     232     // dmpConfig2[]
#define MPU9150_DMP_UPDATES_SIZE    140     // dmpUpdates[]

// after conversion
//#define MPU6050_MPU9150_DMP_CODE_SIZE       1929    // dmpMemory[]
//#define MPU6050_MPU9150_DMP_CONFIG_SIZE     192     // dmpConfig[]
//#define MPU6050_MPU9150_DMP_UPDATES_SIZE    47      // dmpUpdates[]
//#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CODE_SIZE       1417    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]


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

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const unsigned char dmpMemoryMPU6050_MPU9150[MPU6050_MPU9150_DMP_CODE_SIZE] PROGMEM = {
  0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,
};

const unsigned char dmpMemoryMPU6050[MPU6050_DMP_CODE_SIZE] PROGMEM = {
    // bank 0, 256 bytes
  /*  0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,
*/
    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

#ifndef MPU6050_MPU9150_DMP_FIFO_RATE_DIVISOR 
#define MPU6050_MPU9150_DMP_FIFO_RATE_DIVISOR 0x01
#endif

// thanks to Noah Zerkin for piecing this stuff together!
const unsigned char dmpConfigMPU6050[MPU6050_DMP_CONFIG_SIZE] PROGMEM = {
//  BANK    OFFSET  LENGTH  [DATA]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,   // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,   // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,         // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,               // D_0_108 inv_set_accel_calibration
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_22
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                     // D_0_163 inv_set_dead_zone
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                     // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x46,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x01//MPU6050_MPU9150_DMP_FIFO_RATE_DIVISOR // D_0_22 inv_set_fifo_rate
    // 0x03
    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

const unsigned char dmpUpdatesMPU6050[MPU6050_DMP_UPDATES_SIZE] PROGMEM = {
    0x01,   0xB2,   0x02,   0xFF, 0xFF,
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x01,   0x62,   0x02,   0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};


#ifdef MPU_DEBUG
    #define MPU_DEBUG_PRINT(x) Serial.print(x)
    #define MPU_DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define MPU_DEBUG_PRINTLN(x) Serial.println(x)
    #define MPU_DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define MPU_DEBUG_PRINT(x)
    #define MPU_DEBUG_PRINTF(x, y)
    #define MPU_DEBUG_PRINTLN(x)
    #define MPU_DEBUG_PRINTLNF(x, y)
#endif

#define MPU9150_DMP_CODE_SIZE       1962    // dmpMemory[]
#define MPU9150_DMP_CONFIG_SIZE     232     // dmpConfig[]
#define MPU9150_DMP_UPDATES_SIZE    140     // dmpUpdates[]

/* ================================================================================================ *
 | Default MotionApps v4.1 48-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][MAG X ][MAG Y ][MAG Z ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ] |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const unsigned char dmpMemory2[MPU9150_DMP_CODE_SIZE] PROGMEM = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,
    
    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x78, 0xA2,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    
    // bank 3, 256 bytes
    0xD8, 0xDC, 0xF4, 0xD8, 0xB9, 0xAB, 0xF3, 0xF8, 0xFA, 0xF1, 0xBA, 0xA2, 0xDE, 0xB2, 0xB8, 0xB4,
    0xA8, 0x81, 0x98, 0xF7, 0x4A, 0x90, 0x7F, 0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA,
    0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2, 0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80,
    0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF, 0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0,
    0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C, 0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1,
    0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1, 0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3,
    0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01, 0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88,
    0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80, 0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF,
    0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89,
    0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80, 0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9,
    0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E, 0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A,
    0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9, 0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11,
    0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55,
    0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xF0, 0x00, 0x28, 0x50, 0xF5, 0xBA, 0xAD, 0x8F, 0x9F, 0x28, 0x54,
    0x7C, 0xB9, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xDB, 0xB2, 0xB6, 0x8E, 0x9D,
    0xAE, 0xF5, 0x60, 0x68, 0x70, 0xB1, 0xB5, 0xF1, 0xDA, 0xA6, 0xDF, 0xD9, 0xA6, 0xFA, 0xA3, 0x86,
    
    // bank 4, 256 bytes
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    
    // bank 5, 256 bytes
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0x97, 0x86,
    0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97, 0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40,
    0xB9, 0xA3, 0x8A, 0xC3, 0xC5, 0xC7, 0x9A, 0xA3, 0x28, 0x50, 0x78, 0xF1, 0xB5, 0x93, 0x01, 0xD9,
    0xDF, 0xDF, 0xDF, 0xD8, 0xB8, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04, 0x28, 0x51, 0x79, 0x1D, 0x30,
    0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78, 0x78, 0x9B, 0xF1, 0x1A, 0xB0,
    0xF0, 0xB1, 0x83, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0xB0, 0x8B, 0x29, 0x51, 0x79, 0xB1, 0x83, 0x24,

    // bank 6, 256 bytes
    0x70, 0x59, 0xB0, 0x8B, 0x20, 0x58, 0x71, 0xB1, 0x83, 0x44, 0x69, 0x38, 0xB0, 0x8B, 0x39, 0x40,
    0x68, 0xB1, 0x83, 0x64, 0x48, 0x31, 0xB0, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71,
    0x58, 0x44, 0x68, 0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0,
    0x8C, 0xA8, 0x04, 0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02,
    0x26, 0x46, 0x66, 0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38,
    0x64, 0x48, 0x31, 0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19,
    0x31, 0x48, 0x60, 0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86,
    0xA8, 0x6E, 0x76, 0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A,
    0x6E, 0x8A, 0x56, 0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E,
    0x9D, 0xB8, 0xAD, 0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55,
    0x7D, 0x81, 0x91, 0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D,
    0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51,
    0xD9, 0x04, 0xAE, 0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19,
    0x81, 0xAD, 0xD9, 0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9,
    0xAD, 0xAD, 0xAD, 0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76,
    0xF3, 0xAC, 0x2E, 0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC,
    
    // bank 7, 170 bytes (remainder)
    0x30, 0x18, 0xA8, 0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24,
    0xF2, 0xB0, 0x89, 0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9,
    0xD8, 0xD8, 0x79, 0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D,
    0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D,
    0x80, 0x25, 0xDA, 0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34,
    0x3C, 0xF3, 0xAB, 0x8B, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xB0, 0x87, 0x9C, 0xB9,
    0xA3, 0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3,
    0xA3, 0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3,
    0xA3, 0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3,
    0xDC, 0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

const unsigned char dmpConfig2[MPU9150_DMP_CONFIG_SIZE] PROGMEM = {
//  BANK    OFFSET  LENGTH  [DATA]
    0x02,   0xEC,   0x04,   0x00, 0x47, 0x7D, 0x1A,   // ?
    0x03,   0x82,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xB2,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCA, 0xE3, 0x09,   // D_0_104 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x86,   0x03,   0x0C, 0xC9, 0x2C,         // FCFG_2 inv_set_accel_calibration
    0x03,   0x90,   0x03,   0x26, 0x46, 0x66,         //   (continued)...FCFG_2 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x40, 0x00,               // D_0_108 inv_set_accel_calibration

    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x40, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x40, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0xC0, 0x00, 0x00, 0x00,   // CPASS_MTX_22

    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x86,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x22,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x00,   0xA3,   0x01,   0x00,                     // ?
    0x04,   0x29,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x07,   0x62,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x9F,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x67,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x68,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x62,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // ?
    0x02,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // ?
    0x07,   0x83,   0x06,   0xC2, 0xCA, 0xC4, 0xA3, 0xA3, 0xA3, // ?
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE, SPECIAL INSTRUCTION
    0x07,   0xA7,   0x01,   0xFE,                     // ?
    0x07,   0x62,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // ?
    0x07,   0x67,   0x01,   0x9A,                     // ?
    0x07,   0x68,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x07,   0x8D,   0x04,   0xF1, 0x28, 0x30, 0x38,   // ??? CFG_12 inv_send_mag -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x03                // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

const unsigned char dmpUpdates2[MPU9150_DMP_UPDATES_SIZE] PROGMEM = {
    0x01,   0xB2,   0x02,   0xFF, 0xF5,
    0x01,   0x90,   0x04,   0x0A, 0x0D, 0x97, 0xC0,
    0x00,   0xA3,   0x01,   0x00,
    0x04,   0x29,   0x04,   0x87, 0x2D, 0x35, 0x3D,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x02,   0x60,   0x0C,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01,   0x08,   0x02,   0x01, 0x20,
    0x01,   0x0A,   0x02,   0x00, 0x4E,
    0x01,   0x02,   0x02,   0xFE, 0xB3,
    0x02,   0x6C,   0x04,   0x00, 0x00, 0x00, 0x00, // READ
    0x02,   0x6C,   0x04,   0xFA, 0xFE, 0x00, 0x00,
    0x02,   0x60,   0x0C,   0xFF, 0xFF, 0xCB, 0x4D, 0x00, 0x01, 0x08, 0xC1, 0xFF, 0xFF, 0xBC, 0x2C,
    0x02,   0xF4,   0x04,   0x00, 0x00, 0x00, 0x00,
    0x02,   0xF8,   0x04,   0x00, 0x00, 0x00, 0x00,
    0x02,   0xFC,   0x04,   0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};

uint8_t MPU6050_MPU9150::dmpInitialize() {

            // reset device
    MPU_DEBUG_PRINTLN(F("\n\nResetting MPU6050_MPU9150..."));
    reset();
    delay(30); // wait after reset

    // enable sleep mode and wake cycle
    /*Serial.println(F("Enabling sleep mode..."));
    setSleepEnabled(true);
    Serial.println(F("Enabling wake cycle..."));
    setWakeCycleEnabled(true);*/

    // disable sleep mode
    MPU_DEBUG_PRINTLN(F("Disabling sleep mode..."));
    setSleepEnabled(false);

    // get MPU hardware revision
    MPU_DEBUG_PRINTLN(F("Selecting user bank 16..."));
    if(isMPU9150) {
      setMemoryBank(0x10, true, true);
      MPU_DEBUG_PRINTLN(F("Selecting memory byte 6..."));
      //setMemoryStartAddress(0x06);
      // setMemoryBank(0x10, true, false);  // Instead of true, true
      setMemoryStartAddress(0x06);
    } else {
      setMemoryBank(0, false, false);
    }
    MPU_DEBUG_PRINTLN(F("Checking hardware revision..."));
    MPU_DEBUG_PRINT(F("Revision @ user[16][6] = "));
    MPU_DEBUG_PRINTLNF(readMemoryByte(), HEX);
    MPU_DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
    if(isMPU9150) {
      setMemoryBank(0x10, true, false);  // Instead of true, true
    } else {
      setMemoryBank(0, false, false);
    }
    // check OTP bank valid
    MPU_DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
    MPU_DEBUG_PRINT(F("OTP bank is "));
    MPU_DEBUG_PRINTLN(getOTPBankValid() ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    MPU_DEBUG_PRINTLN(F("Reading gyro offset TC values..."));
    int8_t xgOffsetTC = getXGyroOffsetTC();
    int8_t ygOffsetTC = getYGyroOffsetTC();
    int8_t zgOffsetTC = getZGyroOffsetTC();
    MPU_DEBUG_PRINT(F("X gyro offset = "));
    MPU_DEBUG_PRINTLN(xgOffsetTC);
    MPU_DEBUG_PRINT(F("Y gyro offset = "));
    MPU_DEBUG_PRINTLN(ygOffsetTC);
    MPU_DEBUG_PRINT(F("Z gyro offset = "));
    MPU_DEBUG_PRINTLN(zgOffsetTC);


    if(isMPU9150) {
        // setup weird slave stuff (?)
        MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
        setSlaveAddress(0, 0x7F);
        MPU_DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
        setI2CMasterModeEnabled(false);
        MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
        setSlaveAddress(0, 0x68);
        MPU_DEBUG_PRINTLN(F("Resetting I2C Master control..."));
        resetI2CMaster();
        delay(20);

        I2Cdev::readByte(devAddr, MPU6050_MPU9150_RA_USER_CTRL, buffer); // ?
        
        MPU_DEBUG_PRINTLN(F("Enabling interrupt latch, clear on any read, AUX bypass enabled"));
        I2Cdev::writeByte(devAddr, MPU6050_MPU9150_RA_INT_PIN_CFG, 0x32);
    
        // enable MPU AUX I2C bypass mode
        //MPU_DEBUG_PRINTLN(F("Enabling AUX I2C bypass mode..."));
        //setI2CBypassEnabled(true);
    
        MPU_DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
        //mag -> setMode(0);
        I2Cdev::writeByte(0x0E, 0x0A, 0x00);
    
        MPU_DEBUG_PRINTLN(F("Setting magnetometer mode to fuse access..."));
        //mag -> setMode(0x0F);
        I2Cdev::writeByte(0x0E, 0x0A, 0x0F);
    
        MPU_DEBUG_PRINTLN(F("Reading mag magnetometer factory calibration..."));
        int8_t asax, asay, asaz;
        //mag -> getAdjustment(&asax, &asay, &asaz);
        I2Cdev::readBytes(0x0E, 0x10, 3, buffer);
        asax = (int8_t)buffer[0];
        asay = (int8_t)buffer[1];
        asaz = (int8_t)buffer[2];
        MPU_DEBUG_PRINT(F("Adjustment X/Y/Z = "));
        MPU_DEBUG_PRINT(asax);
        MPU_DEBUG_PRINT(F(" / "));
        MPU_DEBUG_PRINT(asay);
        MPU_DEBUG_PRINT(F(" / "));
        MPU_DEBUG_PRINTLN(asaz);
    
        MPU_DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
        //mag -> setMode(0);
        I2Cdev::writeByte(0x0E, 0x0A, 0x00);
    
    }else {


        // setup weird slave stuff (?)
        MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
        setSlaveAddress(0, 0x7F);
        MPU_DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
        setI2CMasterModeEnabled(false);
        MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
        setSlaveAddress(0, 0x68);
        MPU_DEBUG_PRINTLN(F("Resetting I2C Master control..."));
        resetI2CMaster();
        delay(20);
    }
    // load DMP code into memory banks
    MPU_DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
    MPU_DEBUG_PRINTLN(F(" bytes)"));
    
    const unsigned char * dmpUpdates;

    bool writeProgMemoryBlockRes = false;
    if(isMPU9150) {
        MPU_DEBUG_PRINT("MPU9150 - skipping");
        // may be removed
         dmpUpdates = dmpUpdatesMPU6050;
        MPU_DEBUG_PRINT(F("MPU6050_DMP_CODE_SIZE: "));
        MPU_DEBUG_PRINTLN(MPU6050_DMP_CODE_SIZE);
        writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050_MPU9150, MPU6050_MPU9150_DMP_CODE_SIZE,0,0,true);//MPU6050_DMP_CODE_SIZE);
        MPU_DEBUG_PRINT(F("writeProgMemoryBlockRes-1:"));
        MPU_DEBUG_PRINTLN(writeProgMemoryBlockRes);
        writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050, MPU6050_DMP_CODE_SIZE,2,MPU6050_MPU9150_DMP_CODE_SIZE,true);
        //writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050, MPU6050_DMP_CODE_SIZE);
        MPU_DEBUG_PRINT(F("writeProgMemoryBlockRes-2:"));
        MPU_DEBUG_PRINTLN(writeProgMemoryBlockRes);
        
    } else {
        dmpUpdates = dmpUpdatesMPU6050;
        MPU_DEBUG_PRINT(F("MPU6050_DMP_CODE_SIZE: "));
        MPU_DEBUG_PRINTLN(MPU6050_DMP_CODE_SIZE);
        writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050_MPU9150, MPU6050_MPU9150_DMP_CODE_SIZE,0,0,true);//MPU6050_DMP_CODE_SIZE);
        MPU_DEBUG_PRINT(F("writeProgMemoryBlockRes-1:"));
        MPU_DEBUG_PRINTLN(writeProgMemoryBlockRes);
        writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050, MPU6050_DMP_CODE_SIZE,2,MPU6050_MPU9150_DMP_CODE_SIZE,true);
        //writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050, MPU6050_DMP_CODE_SIZE);
        MPU_DEBUG_PRINT(F("writeProgMemoryBlockRes-2:"));
        MPU_DEBUG_PRINTLN(writeProgMemoryBlockRes);
    }

    if (writeProgMemoryBlockRes) {
        MPU_DEBUG_PRINTLN(F("Success! DMP code written and verified."));

        // write DMP configuration
        MPU_DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
        //MPU_DEBUG_PRINT(MPU6050_MPU9150_DMP_CONFIG_SIZE);
        MPU_DEBUG_PRINTLN(F(" bytes in config def)"));
        bool writeProgDMPConfigurationSetRes = false;
        if(isMPU9150) {
            MPU_DEBUG_PRINTLN("MPU9150 - skipping");
        }else{
            MPU_DEBUG_PRINT("MPU6050");
            MPU_DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
            writeProgDMPConfigurationSetRes = writeProgDMPConfigurationSet(dmpConfigMPU6050, MPU6050_DMP_CONFIG_SIZE);
        }
        if (writeProgDMPConfigurationSetRes) {
            MPU_DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

            MPU_DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
            setClockSource(MPU6050_MPU9150_CLOCK_PLL_ZGYRO);

            MPU_DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
            setIntEnabled(1<<MPU6050_MPU9150_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_MPU9150_INTERRUPT_DMP_INT_BIT);

            MPU_DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
            setRate(4); // 1khz / (1 + 4) = 200 Hz

            MPU_DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
            setExternalFrameSync(MPU6050_MPU9150_EXT_SYNC_TEMP_OUT_L);

            MPU_DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
            setDLPFMode(MPU6050_MPU9150_DLPF_BW_42);

            MPU_DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
            setFullScaleGyroRange(MPU6050_MPU9150_GYRO_FS_2000);

            MPU_DEBUG_PRINTLN(F("Setting DMP programm start address"));
            //write start address MSB into register
            setDMPConfig1(0x03);
            //write start address LSB into register
            setDMPConfig2(0x00);

            MPU_DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
            setOTPBankValid(false);

            MPU_DEBUG_PRINTLN(F("Setting X/Y/Z gyro offset TCs to previous values..."));
            setXGyroOffsetTC(xgOffsetTC);
            setYGyroOffsetTC(ygOffsetTC);
            setZGyroOffsetTC(zgOffsetTC);

            //MPU_DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
            //setXGyroOffset(0);
            //setYGyroOffset(0);
            //setZGyroOffset(0);

            MPU_DEBUG_PRINTLN(F("Writing final memory update 1/7 (function unknown)..."));
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Resetting FIFO..."));
            resetFIFO();

            MPU_DEBUG_PRINTLN(F("Reading FIFO count..."));
            uint16_t fifoCount = getFIFOCount();
            uint8_t fifoBuffer[128];

            MPU_DEBUG_PRINT(F("Current FIFO count="));
            MPU_DEBUG_PRINTLN(fifoCount);
            if(isMPU9150){
                MPU_DEBUG_PRINTLN("MPU9150 - skipping");
            }else{
                getFIFOBytes(fifoBuffer, fifoCount);
            }
            MPU_DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
            setMotionDetectionThreshold(2);

            MPU_DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
            setZeroMotionDetectionThreshold(156);

            MPU_DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
            setMotionDetectionDuration(80);

            MPU_DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
            setZeroMotionDetectionDuration(0);

            if(isMPU9150) {
                MPU_DEBUG_PRINTLN("MPU9150 - skipping");
            }

            MPU_DEBUG_PRINTLN(F("Resetting FIFO..."));
            if(isMPU9150){
                MPU_DEBUG_PRINTLN("MPU9150 - skipping");
            }else{
                resetFIFO();

                MPU_DEBUG_PRINTLN(F("Enabling FIFO..."));
                setFIFOEnabled(true);

                MPU_DEBUG_PRINTLN(F("Enabling DMP..."));
                setDMPEnabled(true);

                MPU_DEBUG_PRINTLN(F("Resetting DMP..."));
                resetDMP();
            
    // e1
                MPU_DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
                for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                MPU_DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
                for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                MPU_DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
                for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                MPU_DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
                while ((fifoCount = getFIFOCount()) < 3);

                MPU_DEBUG_PRINT(F("Current FIFO count="));
                MPU_DEBUG_PRINTLN(fifoCount);
                MPU_DEBUG_PRINTLN(F("Reading FIFO data..."));
                getFIFOBytes(fifoBuffer, fifoCount);

                MPU_DEBUG_PRINTLN(F("Reading interrupt status..."));

                MPU_DEBUG_PRINT(F("Current interrupt status="));
                MPU_DEBUG_PRINTLNF(getIntStatus(), HEX);

                MPU_DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
                for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                MPU_DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
                while ((fifoCount = getFIFOCount()) < 3);

                MPU_DEBUG_PRINT(F("Current FIFO count="));
                MPU_DEBUG_PRINTLN(fifoCount);

                MPU_DEBUG_PRINTLN(F("Reading FIFO data..."));
                getFIFOBytes(fifoBuffer, fifoCount);

                MPU_DEBUG_PRINTLN(F("Reading interrupt status..."));

                MPU_DEBUG_PRINT(F("Current interrupt status="));
                MPU_DEBUG_PRINTLNF(getIntStatus(), HEX);

                MPU_DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
                for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            }
// e2
            MPU_DEBUG_PRINTLN(F("DMP is good to go! Finally."));

            MPU_DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
            setDMPEnabled(false);

            MPU_DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
            if(isMPU9150){
                dmpPacketSize = 48;
            }else{
                dmpPacketSize = 42;
            }
            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

            MPU_DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
            resetFIFO();
            getIntStatus();
        } else {
            MPU_DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
            return 2; // configuration block loading failed
        }
    } else {
        MPU_DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
        return 1; // main binary block loading failed
    }
    return 0; // success
}




uint8_t MPU6050_MPU9150::dmpInitialize3() {

            // reset device
    // MPU_DEBUG_PRINTLN(F("\n\nResetting MPU6050_MPU9150..."));
    // reset();
    // delay(30); // wait after reset

    // enable sleep mode and wake cycle
    /*Serial.println(F("Enabling sleep mode..."));
    setSleepEnabled(true);
    Serial.println(F("Enabling wake cycle..."));
    setWakeCycleEnabled(true);*/

    // disable sleep mode
    // MPU_DEBUG_PRINTLN(F("Disabling sleep mode..."));
    // setSleepEnabled(false);

    // get MPU hardware revision
    // MPU_DEBUG_PRINTLN(F("Selecting user bank 16..."));
      // setMemoryBank(0, false, false);
    // MPU_DEBUG_PRINTLN(F("Checking hardware revision..."));
    // MPU_DEBUG_PRINT(F("Revision @ user[16][6] = "));
    // MPU_DEBUG_PRINTLNF(readMemoryByte(), HEX);
    // MPU_DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
      // setMemoryBank(0, false, false);
    // check OTP bank valid
    // MPU_DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
    // MPU_DEBUG_PRINT(F("OTP bank is "));
    // MPU_DEBUG_PRINTLN(getOTPBankValid() ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    // MPU_DEBUG_PRINTLN(F("Reading gyro offset TC values..."));
    // int8_t xgOffsetTC = getXGyroOffsetTC();
    // int8_t ygOffsetTC = getYGyroOffsetTC();
    // int8_t zgOffsetTC = getZGyroOffsetTC();
    // MPU_DEBUG_PRINT(F("X gyro offset = "));
    // MPU_DEBUG_PRINTLN(xgOffsetTC);
    // MPU_DEBUG_PRINT(F("Y gyro offset = "));
    // MPU_DEBUG_PRINTLN(ygOffsetTC);
    // MPU_DEBUG_PRINT(F("Z gyro offset = "));
    // MPU_DEBUG_PRINTLN(zgOffsetTC);


   
        // setup weird slave stuff (?)
        // MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
        // setSlaveAddress(0, 0x7F);
        // MPU_DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
        // setI2CMasterModeEnabled(false);
        // MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
        // setSlaveAddress(0, 0x68);
        // MPU_DEBUG_PRINTLN(F("Resetting I2C Master control..."));
        // resetI2CMaster();
        // delay(20);


    // load DMP code into memory banks
    // MPU_DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
    // MPU_DEBUG_PRINTLN(F(" bytes)"));
    
    const unsigned char * dmpUpdates;

    bool writeProgMemoryBlockRes = false;
   
        // dmpUpdates = dmpUpdatesMPU6050;
        // MPU_DEBUG_PRINT(F("MPU6050_DMP_CODE_SIZE: "));
        // MPU_DEBUG_PRINTLN(MPU6050_DMP_CODE_SIZE);
        // writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050_MPU9150, MPU6050_MPU9150_DMP_CODE_SIZE,0,0,true);//MPU6050_DMP_CODE_SIZE);
        // MPU_DEBUG_PRINT(F("writeProgMemoryBlockRes-1:"));
        // MPU_DEBUG_PRINTLN(writeProgMemoryBlockRes);
        // writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050, MPU6050_DMP_CODE_SIZE,2,MPU6050_MPU9150_DMP_CODE_SIZE,true);
        // //writeProgMemoryBlockRes = writeProgMemoryBlock(dmpMemoryMPU6050, MPU6050_DMP_CODE_SIZE);
        // MPU_DEBUG_PRINT(F("writeProgMemoryBlockRes-2:"));
        // MPU_DEBUG_PRINTLN(writeProgMemoryBlockRes);

//     if (writeProgMemoryBlockRes) {
//         MPU_DEBUG_PRINTLN(F("Success! DMP code written and verified."));

//         // write DMP configuration
//         MPU_DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
//         //MPU_DEBUG_PRINT(MPU6050_MPU9150_DMP_CONFIG_SIZE);
//         MPU_DEBUG_PRINTLN(F(" bytes in config def)"));
//         bool writeProgDMPConfigurationSetRes = false;
//             MPU_DEBUG_PRINT("MPU6050");
//             MPU_DEBUG_PRINT(MPU6050_DMP_CODE_SIZE);
//             writeProgDMPConfigurationSetRes = writeProgDMPConfigurationSet(dmpConfigMPU6050, MPU6050_DMP_CONFIG_SIZE);
//         if (writeProgDMPConfigurationSetRes) {
//             MPU_DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

//             MPU_DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
//             setClockSource(MPU6050_MPU9150_CLOCK_PLL_ZGYRO);

//             MPU_DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
//             setIntEnabled(1<<MPU6050_MPU9150_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_MPU9150_INTERRUPT_DMP_INT_BIT);

//             MPU_DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
//             setRate(4); // 1khz / (1 + 4) = 200 Hz

//             MPU_DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
//             setExternalFrameSync(MPU6050_MPU9150_EXT_SYNC_TEMP_OUT_L);

//             MPU_DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
//             setDLPFMode(MPU6050_MPU9150_DLPF_BW_42);

//             MPU_DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
//             setFullScaleGyroRange(MPU6050_MPU9150_GYRO_FS_2000);

//             MPU_DEBUG_PRINTLN(F("Setting DMP programm start address"));
//             //write start address MSB into register
//             setDMPConfig1(0x03);
//             //write start address LSB into register
//             setDMPConfig2(0x00);

//             MPU_DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
//             setOTPBankValid(false);

//             MPU_DEBUG_PRINTLN(F("Setting X/Y/Z gyro offset TCs to previous values..."));
//             setXGyroOffsetTC(xgOffsetTC);
//             setYGyroOffsetTC(ygOffsetTC);
//             setZGyroOffsetTC(zgOffsetTC);

//             //MPU_DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
//             //setXGyroOffset(0);
//             //setYGyroOffset(0);
//             //setZGyroOffset(0);

//             MPU_DEBUG_PRINTLN(F("Writing final memory update 1/7 (function unknown)..."));
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             MPU_DEBUG_PRINTLN(F("Writing final memory update 2/7 (function unknown)..."));
//             for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
//             writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

//             MPU_DEBUG_PRINTLN(F("Resetting FIFO..."));
//             resetFIFO();

//             MPU_DEBUG_PRINTLN(F("Reading FIFO count..."));
                     setDMPEnabled(true);
   uint16_t fifoCount = getFIFOCount();
            uint8_t fifoBuffer[128];

//             MPU_DEBUG_PRINT(F("Current FIFO count="));
//             MPU_DEBUG_PRINTLN(fifoCount);
//             if(isMPU9150){
//                 MPU_DEBUG_PRINTLN("MPU9150 - skipping");
//             }else{
//                 getFIFOBytes(fifoBuffer, fifoCount);
//             }
//             MPU_DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
//             setMotionDetectionThreshold(2);

//             MPU_DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
//             setZeroMotionDetectionThreshold(156);

//             MPU_DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
//             setMotionDetectionDuration(80);

//             MPU_DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
//             setZeroMotionDetectionDuration(0);

//             if(isMPU9150) {
//                 MPU_DEBUG_PRINTLN("MPU9150 - skipping");
//             }

//             MPU_DEBUG_PRINTLN(F("Resetting FIFO..."));
//                 resetFIFO();

//                 MPU_DEBUG_PRINTLN(F("Enabling FIFO..."));
//                 setFIFOEnabled(true);

//                 MPU_DEBUG_PRINTLN(F("Enabling DMP..."));
//                 setDMPEnabled(true);

//                 MPU_DEBUG_PRINTLN(F("Resetting DMP..."));
//                 resetDMP();
            
//     // e1
                // MPU_DEBUG_PRINTLN(F("Writing final memory update 3/7 (function unknown)..."));
                // for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                // writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                // // MPU_DEBUG_PRINTLN(F("Writing final memory update 4/7 (function unknown)..."));
                // for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                // writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                // // MPU_DEBUG_PRINTLN(F("Writing final memory update 5/7 (function unknown)..."));
                // for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                // writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                // MPU_DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
                // while ((fifoCount = getFIFOCount()) < 3);

                // MPU_DEBUG_PRINT(F("Current FIFO count="));
                // MPU_DEBUG_PRINTLN(fifoCount);
                // MPU_DEBUG_PRINTLN(F("Reading FIFO data..."));
                // getFIFOBytes(fifoBuffer, fifoCount);

                // MPU_DEBUG_PRINTLN(F("Reading interrupt status..."));

                // MPU_DEBUG_PRINT(F("Current interrupt status="));
                // MPU_DEBUG_PRINTLNF(getIntStatus(), HEX);
                // getIntStatus();

                // MPU_DEBUG_PRINTLN(F("Reading final memory update 6/7 (function unknown)..."));
                // for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                // readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

                // MPU_DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
                // while ((fifoCount = getFIFOCount()) < 3);

                // MPU_DEBUG_PRINT(F("Current FIFO count="));
                // MPU_DEBUG_PRINTLN(fifoCount);

                // MPU_DEBUG_PRINTLN(F("Reading FIFO data..."));
                // getFIFOBytes(fifoBuffer, fifoCount);

                // MPU_DEBUG_PRINTLN(F("Reading interrupt status..."));

                // MPU_DEBUG_PRINT(F("Current interrupt status="));
                // MPU_DEBUG_PRINTLNF(getIntStatus(), HEX);

                // MPU_DEBUG_PRINTLN(F("Writing final memory update 7/7 (function unknown)..."));
                // for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
                // writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
// e2
            // MPU_DEBUG_PRINTLN(F("DMP is good to go! Finally."));

//             MPU_DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
//             setDMPEnabled(false);

//             MPU_DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
//                 dmpPacketSize = 42;
//             /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
//                 return 3; // TODO: proper error code for no memory
//             }*/

  // MPU_DEBUG_PRINTLN(F("Waiting for FIFO count > 2..."));
                // while ((fifoCount = getFIFOCount()) < 3);

            // MPU_DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
            resetFIFO();
            getIntStatus();
//         } else {
//             MPU_DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
//             return 2; // configuration block loading failed
//         }
    // } else {
        // MPU_DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
        // return 1; // main binary block loading failed
    // }
    return 0; // success
}



//Magnetometer Registers
#define MPU9150_RA_MAG_ADDRESS		0x0C
#define MPU9150_RA_MAG_XOUT_L		0x03
#define MPU9150_RA_MAG_XOUT_H		0x04
#define MPU9150_RA_MAG_YOUT_L		0x05
#define MPU9150_RA_MAG_YOUT_H		0x06
#define MPU9150_RA_MAG_ZOUT_L		0x07
#define MPU9150_RA_MAG_ZOUT_H		0x08

#define MPU9150_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU9150_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU9150_DEFAULT_ADDRESS     MPU9150_ADDRESS_AD0_LOW

#define MPU9150_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9150_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9150_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9150_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU9150_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU9150_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU9150_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU9150_RA_XA_OFFS_L_TC     0x07
#define MPU9150_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU9150_RA_YA_OFFS_L_TC     0x09
#define MPU9150_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU9150_RA_ZA_OFFS_L_TC     0x0B
#define MPU9150_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9150_RA_XG_OFFS_USRL     0x14
#define MPU9150_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9150_RA_YG_OFFS_USRL     0x16
#define MPU9150_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU9150_RA_ZG_OFFS_USRL     0x18
#define MPU9150_RA_SMPLRT_DIV       0x19
#define MPU9150_RA_CONFIG           0x1A
#define MPU9150_RA_GYRO_CONFIG      0x1B
#define MPU9150_RA_ACCEL_CONFIG     0x1C
#define MPU9150_RA_FF_THR           0x1D
#define MPU9150_RA_FF_DUR           0x1E
#define MPU9150_RA_MOT_THR          0x1F
#define MPU9150_RA_MOT_DUR          0x20
#define MPU9150_RA_ZRMOT_THR        0x21
#define MPU9150_RA_ZRMOT_DUR        0x22
#define MPU9150_RA_FIFO_EN          0x23
#define MPU9150_RA_I2C_MST_CTRL     0x24
#define MPU9150_RA_I2C_SLV0_ADDR    0x25
#define MPU9150_RA_I2C_SLV0_REG     0x26
#define MPU9150_RA_I2C_SLV0_CTRL    0x27
#define MPU9150_RA_I2C_SLV1_ADDR    0x28
#define MPU9150_RA_I2C_SLV1_REG     0x29
#define MPU9150_RA_I2C_SLV1_CTRL    0x2A
#define MPU9150_RA_I2C_SLV2_ADDR    0x2B
#define MPU9150_RA_I2C_SLV2_REG     0x2C
#define MPU9150_RA_I2C_SLV2_CTRL    0x2D
#define MPU9150_RA_I2C_SLV3_ADDR    0x2E
#define MPU9150_RA_I2C_SLV3_REG     0x2F
#define MPU9150_RA_I2C_SLV3_CTRL    0x30
#define MPU9150_RA_I2C_SLV4_ADDR    0x31
#define MPU9150_RA_I2C_SLV4_REG     0x32
#define MPU9150_RA_I2C_SLV4_DO      0x33
#define MPU9150_RA_I2C_SLV4_CTRL    0x34
#define MPU9150_RA_I2C_SLV4_DI      0x35
#define MPU9150_RA_I2C_MST_STATUS   0x36
#define MPU9150_RA_INT_PIN_CFG      0x37
#define MPU9150_RA_INT_ENABLE       0x38
#define MPU9150_RA_DMP_INT_STATUS   0x39
#define MPU9150_RA_INT_STATUS       0x3A
#define MPU9150_RA_ACCEL_XOUT_H     0x3B
#define MPU9150_RA_ACCEL_XOUT_L     0x3C
#define MPU9150_RA_ACCEL_YOUT_H     0x3D
#define MPU9150_RA_ACCEL_YOUT_L     0x3E
#define MPU9150_RA_ACCEL_ZOUT_H     0x3F
#define MPU9150_RA_ACCEL_ZOUT_L     0x40
#define MPU9150_RA_TEMP_OUT_H       0x41
#define MPU9150_RA_TEMP_OUT_L       0x42
#define MPU9150_RA_GYRO_XOUT_H      0x43
#define MPU9150_RA_GYRO_XOUT_L      0x44
#define MPU9150_RA_GYRO_YOUT_H      0x45
#define MPU9150_RA_GYRO_YOUT_L      0x46
#define MPU9150_RA_GYRO_ZOUT_H      0x47
#define MPU9150_RA_GYRO_ZOUT_L      0x48
#define MPU9150_RA_EXT_SENS_DATA_00 0x49
#define MPU9150_RA_EXT_SENS_DATA_01 0x4A
#define MPU9150_RA_EXT_SENS_DATA_02 0x4B
#define MPU9150_RA_EXT_SENS_DATA_03 0x4C
#define MPU9150_RA_EXT_SENS_DATA_04 0x4D
#define MPU9150_RA_EXT_SENS_DATA_05 0x4E
#define MPU9150_RA_EXT_SENS_DATA_06 0x4F
#define MPU9150_RA_EXT_SENS_DATA_07 0x50
#define MPU9150_RA_EXT_SENS_DATA_08 0x51
#define MPU9150_RA_EXT_SENS_DATA_09 0x52
#define MPU9150_RA_EXT_SENS_DATA_10 0x53
#define MPU9150_RA_EXT_SENS_DATA_11 0x54
#define MPU9150_RA_EXT_SENS_DATA_12 0x55
#define MPU9150_RA_EXT_SENS_DATA_13 0x56
#define MPU9150_RA_EXT_SENS_DATA_14 0x57
#define MPU9150_RA_EXT_SENS_DATA_15 0x58
#define MPU9150_RA_EXT_SENS_DATA_16 0x59
#define MPU9150_RA_EXT_SENS_DATA_17 0x5A
#define MPU9150_RA_EXT_SENS_DATA_18 0x5B
#define MPU9150_RA_EXT_SENS_DATA_19 0x5C
#define MPU9150_RA_EXT_SENS_DATA_20 0x5D
#define MPU9150_RA_EXT_SENS_DATA_21 0x5E
#define MPU9150_RA_EXT_SENS_DATA_22 0x5F
#define MPU9150_RA_EXT_SENS_DATA_23 0x60
#define MPU9150_RA_MOT_DETECT_STATUS    0x61
#define MPU9150_RA_I2C_SLV0_DO      0x63
#define MPU9150_RA_I2C_SLV1_DO      0x64
#define MPU9150_RA_I2C_SLV2_DO      0x65
#define MPU9150_RA_I2C_SLV3_DO      0x66
#define MPU9150_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU9150_RA_SIGNAL_PATH_RESET    0x68
#define MPU9150_RA_MOT_DETECT_CTRL      0x69
#define MPU9150_RA_USER_CTRL        0x6A
#define MPU9150_RA_PWR_MGMT_1       0x6B
#define MPU9150_RA_PWR_MGMT_2       0x6C
#define MPU9150_RA_BANK_SEL         0x6D
#define MPU9150_RA_MEM_START_ADDR   0x6E
#define MPU9150_RA_MEM_R_W          0x6F
#define MPU9150_RA_DMP_CFG_1        0x70
#define MPU9150_RA_DMP_CFG_2        0x71
#define MPU9150_RA_FIFO_COUNTH      0x72
#define MPU9150_RA_FIFO_COUNTL      0x73
#define MPU9150_RA_FIFO_R_W         0x74
#define MPU9150_RA_WHO_AM_I         0x75

#define MPU9150_TC_PWR_MODE_BIT     7
#define MPU9150_TC_OFFSET_BIT       6
#define MPU9150_TC_OFFSET_LENGTH    6
#define MPU9150_TC_OTP_BNK_VLD_BIT  0

#define MPU9150_VDDIO_LEVEL_VLOGIC  0
#define MPU9150_VDDIO_LEVEL_VDD     1

#define MPU9150_CFG_EXT_SYNC_SET_BIT    5
#define MPU9150_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU9150_CFG_DLPF_CFG_BIT    2
#define MPU9150_CFG_DLPF_CFG_LENGTH 3

#define MPU9150_EXT_SYNC_DISABLED       0x0
#define MPU9150_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU9150_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU9150_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU9150_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU9150_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU9150_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU9150_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU9150_DLPF_BW_256         0x00
#define MPU9150_DLPF_BW_188         0x01
#define MPU9150_DLPF_BW_98          0x02
#define MPU9150_DLPF_BW_42          0x03
#define MPU9150_DLPF_BW_20          0x04
#define MPU9150_DLPF_BW_10          0x05
#define MPU9150_DLPF_BW_5           0x06

#define MPU9150_GCONFIG_FS_SEL_BIT      4
#define MPU9150_GCONFIG_FS_SEL_LENGTH   2

#define MPU9150_GYRO_FS_250         0x00
#define MPU9150_GYRO_FS_500         0x01
#define MPU9150_GYRO_FS_1000        0x02
#define MPU9150_GYRO_FS_2000        0x03

#define MPU9150_ACONFIG_XA_ST_BIT           7
#define MPU9150_ACONFIG_YA_ST_BIT           6
#define MPU9150_ACONFIG_ZA_ST_BIT           5
#define MPU9150_ACONFIG_AFS_SEL_BIT         4
#define MPU9150_ACONFIG_AFS_SEL_LENGTH      2
#define MPU9150_ACONFIG_ACCEL_HPF_BIT       2
#define MPU9150_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU9150_ACCEL_FS_2          0x00
#define MPU9150_ACCEL_FS_4          0x01
#define MPU9150_ACCEL_FS_8          0x02
#define MPU9150_ACCEL_FS_16         0x03

#define MPU9150_DHPF_RESET          0x00
#define MPU9150_DHPF_5              0x01
#define MPU9150_DHPF_2P5            0x02
#define MPU9150_DHPF_1P25           0x03
#define MPU9150_DHPF_0P63           0x04
#define MPU9150_DHPF_HOLD           0x07

#define MPU9150_TEMP_FIFO_EN_BIT    7
#define MPU9150_XG_FIFO_EN_BIT      6
#define MPU9150_YG_FIFO_EN_BIT      5
#define MPU9150_ZG_FIFO_EN_BIT      4
#define MPU9150_ACCEL_FIFO_EN_BIT   3
#define MPU9150_SLV2_FIFO_EN_BIT    2
#define MPU9150_SLV1_FIFO_EN_BIT    1
#define MPU9150_SLV0_FIFO_EN_BIT    0

#define MPU9150_MULT_MST_EN_BIT     7
#define MPU9150_WAIT_FOR_ES_BIT     6
#define MPU9150_SLV_3_FIFO_EN_BIT   5
#define MPU9150_I2C_MST_P_NSR_BIT   4
#define MPU9150_I2C_MST_CLK_BIT     3
#define MPU9150_I2C_MST_CLK_LENGTH  4

#define MPU9150_CLOCK_DIV_348       0x0
#define MPU9150_CLOCK_DIV_333       0x1
#define MPU9150_CLOCK_DIV_320       0x2
#define MPU9150_CLOCK_DIV_308       0x3
#define MPU9150_CLOCK_DIV_296       0x4
#define MPU9150_CLOCK_DIV_286       0x5
#define MPU9150_CLOCK_DIV_276       0x6
#define MPU9150_CLOCK_DIV_267       0x7
#define MPU9150_CLOCK_DIV_258       0x8
#define MPU9150_CLOCK_DIV_500       0x9
#define MPU9150_CLOCK_DIV_471       0xA
#define MPU9150_CLOCK_DIV_444       0xB
#define MPU9150_CLOCK_DIV_421       0xC
#define MPU9150_CLOCK_DIV_400       0xD
#define MPU9150_CLOCK_DIV_381       0xE
#define MPU9150_CLOCK_DIV_364       0xF

#define MPU9150_I2C_SLV_RW_BIT      7
#define MPU9150_I2C_SLV_ADDR_BIT    6
#define MPU9150_I2C_SLV_ADDR_LENGTH 7
#define MPU9150_I2C_SLV_EN_BIT      7
#define MPU9150_I2C_SLV_BYTE_SW_BIT 6
#define MPU9150_I2C_SLV_REG_DIS_BIT 5
#define MPU9150_I2C_SLV_GRP_BIT     4
#define MPU9150_I2C_SLV_LEN_BIT     3
#define MPU9150_I2C_SLV_LEN_LENGTH  4

#define MPU9150_I2C_SLV4_RW_BIT         7
#define MPU9150_I2C_SLV4_ADDR_BIT       6
#define MPU9150_I2C_SLV4_ADDR_LENGTH    7
#define MPU9150_I2C_SLV4_EN_BIT         7
#define MPU9150_I2C_SLV4_INT_EN_BIT     6
#define MPU9150_I2C_SLV4_REG_DIS_BIT    5
#define MPU9150_I2C_SLV4_MST_DLY_BIT    4
#define MPU9150_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU9150_MST_PASS_THROUGH_BIT    7
#define MPU9150_MST_I2C_SLV4_DONE_BIT   6
#define MPU9150_MST_I2C_LOST_ARB_BIT    5
#define MPU9150_MST_I2C_SLV4_NACK_BIT   4
#define MPU9150_MST_I2C_SLV3_NACK_BIT   3
#define MPU9150_MST_I2C_SLV2_NACK_BIT   2
#define MPU9150_MST_I2C_SLV1_NACK_BIT   1
#define MPU9150_MST_I2C_SLV0_NACK_BIT   0

#define MPU9150_INTCFG_INT_LEVEL_BIT        7
#define MPU9150_INTCFG_INT_OPEN_BIT         6
#define MPU9150_INTCFG_LATCH_INT_EN_BIT     5
#define MPU9150_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU9150_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU9150_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU9150_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU9150_INTCFG_CLKOUT_EN_BIT        0

#define MPU9150_INTMODE_ACTIVEHIGH  0x00
#define MPU9150_INTMODE_ACTIVELOW   0x01

#define MPU9150_INTDRV_PUSHPULL     0x00
#define MPU9150_INTDRV_OPENDRAIN    0x01

#define MPU9150_INTLATCH_50USPULSE  0x00
#define MPU9150_INTLATCH_WAITCLEAR  0x01

#define MPU9150_INTCLEAR_STATUSREAD 0x00
#define MPU9150_INTCLEAR_ANYREAD    0x01

#define MPU9150_INTERRUPT_FF_BIT            7
#define MPU9150_INTERRUPT_MOT_BIT           6
#define MPU9150_INTERRUPT_ZMOT_BIT          5
#define MPU9150_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU9150_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU9150_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU9150_INTERRUPT_DMP_INT_BIT       1
#define MPU9150_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU9150_DMPINT_5_BIT            5
#define MPU9150_DMPINT_4_BIT            4
#define MPU9150_DMPINT_3_BIT            3
#define MPU9150_DMPINT_2_BIT            2
#define MPU9150_DMPINT_1_BIT            1
#define MPU9150_DMPINT_0_BIT            0

#define MPU9150_MOTION_MOT_XNEG_BIT     7
#define MPU9150_MOTION_MOT_XPOS_BIT     6
#define MPU9150_MOTION_MOT_YNEG_BIT     5
#define MPU9150_MOTION_MOT_YPOS_BIT     4
#define MPU9150_MOTION_MOT_ZNEG_BIT     3
#define MPU9150_MOTION_MOT_ZPOS_BIT     2
#define MPU9150_MOTION_MOT_ZRMOT_BIT    0

#define MPU9150_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU9150_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU9150_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU9150_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU9150_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU9150_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU9150_PATHRESET_GYRO_RESET_BIT    2
#define MPU9150_PATHRESET_ACCEL_RESET_BIT   1
#define MPU9150_PATHRESET_TEMP_RESET_BIT    0

#define MPU9150_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU9150_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU9150_DETECT_FF_COUNT_BIT             3
#define MPU9150_DETECT_FF_COUNT_LENGTH          2
#define MPU9150_DETECT_MOT_COUNT_BIT            1
#define MPU9150_DETECT_MOT_COUNT_LENGTH         2

#define MPU9150_DETECT_DECREMENT_RESET  0x0
#define MPU9150_DETECT_DECREMENT_1      0x1
#define MPU9150_DETECT_DECREMENT_2      0x2
#define MPU9150_DETECT_DECREMENT_4      0x3

#define MPU9150_USERCTRL_DMP_EN_BIT             7
#define MPU9150_USERCTRL_FIFO_EN_BIT            6
#define MPU9150_USERCTRL_I2C_MST_EN_BIT         5
#define MPU9150_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU9150_USERCTRL_DMP_RESET_BIT          3
#define MPU9150_USERCTRL_FIFO_RESET_BIT         2
#define MPU9150_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU9150_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU9150_PWR1_DEVICE_RESET_BIT   7
#define MPU9150_PWR1_SLEEP_BIT          6
#define MPU9150_PWR1_CYCLE_BIT          5
#define MPU9150_PWR1_TEMP_DIS_BIT       3
#define MPU9150_PWR1_CLKSEL_BIT         2
#define MPU9150_PWR1_CLKSEL_LENGTH      3

#define MPU9150_CLOCK_INTERNAL          0x00
#define MPU9150_CLOCK_PLL_XGYRO         0x01
#define MPU9150_CLOCK_PLL_YGYRO         0x02
#define MPU9150_CLOCK_PLL_ZGYRO         0x03
#define MPU9150_CLOCK_PLL_EXT32K        0x04
#define MPU9150_CLOCK_PLL_EXT19M        0x05
#define MPU9150_CLOCK_KEEP_RESET        0x07

#define MPU9150_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU9150_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU9150_PWR2_STBY_XA_BIT            5
#define MPU9150_PWR2_STBY_YA_BIT            4
#define MPU9150_PWR2_STBY_ZA_BIT            3
#define MPU9150_PWR2_STBY_XG_BIT            2
#define MPU9150_PWR2_STBY_YG_BIT            1
#define MPU9150_PWR2_STBY_ZG_BIT            0

#define MPU9150_WAKE_FREQ_1P25      0x0
#define MPU9150_WAKE_FREQ_2P5       0x1
#define MPU9150_WAKE_FREQ_5         0x2
#define MPU9150_WAKE_FREQ_10        0x3

#define MPU9150_BANKSEL_PRFTCH_EN_BIT       6
#define MPU9150_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU9150_BANKSEL_MEM_SEL_BIT         4
#define MPU9150_BANKSEL_MEM_SEL_LENGTH      5

#define MPU9150_WHO_AM_I_BIT        6
#define MPU9150_WHO_AM_I_LENGTH     6

#define MPU9150_DMP_MEMORY_BANKS        8
#define MPU9150_DMP_MEMORY_BANK_SIZE    256
#define MPU9150_DMP_MEMORY_CHUNK_SIZE   16



//#define DEBUG
#ifdef DEBUG
    #define MPU_DEBUG_PRINT(x) Serial.print(x)
    #define MPU_DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define MPU_DEBUG_PRINTLN(x) Serial.println(x)
    #define MPU_DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define MPU_DEBUG_PRINT(x)
    #define MPU_DEBUG_PRINTF(x, y)
    #define MPU_DEBUG_PRINTLN(x)
    #define MPU_DEBUG_PRINTLNF(x, y)
#endif

// note: DMP code memory blocks defined at end of header file

uint8_t MPU6050_MPU9150::dmpInitialize2() {
    // reset device
    MPU_DEBUG_PRINTLN(F("\n\nResetting MPU9150..."));
    reset();
    delay(30); // wait after reset

    // disable sleep mode
    MPU_DEBUG_PRINTLN(F("Disabling sleep mode..."));
    setSleepEnabled(false);

    // get MPU product ID
    MPU_DEBUG_PRINTLN(F("Getting product ID..."));
    uint8_t productID = 0; //getProductID();
    MPU_DEBUG_PRINT(F("Product ID = "));
    MPU_DEBUG_PRINT(productID);

    // get MPU hardware revision
    MPU_DEBUG_PRINTLN(F("Selecting user bank 16..."));
    setMemoryBank(0x10, true, true);
    MPU_DEBUG_PRINTLN(F("Selecting memory byte 6..."));
    setMemoryStartAddress(0x06);
    MPU_DEBUG_PRINTLN(F("Checking hardware revision..."));
    uint8_t hwRevision = readMemoryByte();
    MPU_DEBUG_PRINT(F("Revision @ user[16][6] = "));
    MPU_DEBUG_PRINTLNF(hwRevision, HEX);
    MPU_DEBUG_PRINTLN(F("Resetting memory bank selection to 0..."));
    setMemoryBank(0, false, false);

    // check OTP bank valid
    MPU_DEBUG_PRINTLN(F("Reading OTP bank valid flag..."));
    uint8_t otpValid = getOTPBankValid();
    MPU_DEBUG_PRINT(F("OTP bank is "));
    MPU_DEBUG_PRINTLN(otpValid ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    MPU_DEBUG_PRINTLN(F("Reading gyro offset values..."));
    int8_t xgOffset = getXGyroOffset();
    int8_t ygOffset = getYGyroOffset();
    int8_t zgOffset = getZGyroOffset();
    MPU_DEBUG_PRINT(F("X gyro offset = "));
    MPU_DEBUG_PRINTLN(xgOffset);
    MPU_DEBUG_PRINT(F("Y gyro offset = "));
    MPU_DEBUG_PRINTLN(ygOffset);
    MPU_DEBUG_PRINT(F("Z gyro offset = "));
    MPU_DEBUG_PRINTLN(zgOffset);

 MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x7F..."));
        setSlaveAddress(0, 0x7F);
        MPU_DEBUG_PRINTLN(F("Disabling I2C Master mode..."));
        setI2CMasterModeEnabled(false);
        MPU_DEBUG_PRINTLN(F("Setting slave 0 address to 0x68 (self)..."));
        setSlaveAddress(0, 0x68);
        MPU_DEBUG_PRINTLN(F("Resetting I2C Master control..."));
        resetI2CMaster();
        delay(20);
    
    
    I2Cdev::readByte(devAddr, MPU9150_RA_USER_CTRL, buffer); // ?
    
    MPU_DEBUG_PRINTLN(F("Enabling interrupt latch, clear on any read, AUX bypass enabled"));
    I2Cdev::writeByte(devAddr, MPU9150_RA_INT_PIN_CFG, 0x32);

    // enable MPU AUX I2C bypass mode
    //MPU_DEBUG_PRINTLN(F("Enabling AUX I2C bypass mode..."));
    //setI2CBypassEnabled(true);

    MPU_DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
    //mag -> setMode(0);
    I2Cdev::writeByte(0x0E, 0x0A, 0x00);

    MPU_DEBUG_PRINTLN(F("Setting magnetometer mode to fuse access..."));
    //mag -> setMode(0x0F);
    I2Cdev::writeByte(0x0E, 0x0A, 0x0F);

    MPU_DEBUG_PRINTLN(F("Reading mag magnetometer factory calibration..."));
    int8_t asax, asay, asaz;
    //mag -> getAdjustment(&asax, &asay, &asaz);
    I2Cdev::readBytes(0x0E, 0x10, 3, buffer);
    asax = (int8_t)buffer[0];
    asay = (int8_t)buffer[1];
    asaz = (int8_t)buffer[2];
    MPU_DEBUG_PRINT(F("Adjustment X/Y/Z = "));
    MPU_DEBUG_PRINT(asax);
    MPU_DEBUG_PRINT(F(" / "));
    MPU_DEBUG_PRINT(asay);
    MPU_DEBUG_PRINT(F(" / "));
    MPU_DEBUG_PRINTLN(asaz);

    MPU_DEBUG_PRINTLN(F("Setting magnetometer mode to power-down..."));
    //mag -> setMode(0);
    I2Cdev::writeByte(0x0E, 0x0A, 0x00);

    // load DMP code into memory banks
    MPU_DEBUG_PRINT(F("Writing DMP code to MPU memory banks ("));
    MPU_DEBUG_PRINT(MPU9150_DMP_CODE_SIZE);
    MPU_DEBUG_PRINTLN(F(" bytes)"));
    if (writeProgMemoryBlock(dmpMemory2, MPU9150_DMP_CODE_SIZE)) {
        MPU_DEBUG_PRINTLN(F("Success! DMP code written and verified."));

        MPU_DEBUG_PRINTLN(F("Configuring DMP and related settings..."));

        // write DMP configuration
        MPU_DEBUG_PRINT(F("Writing DMP configuration to MPU memory banks ("));
        MPU_DEBUG_PRINT(MPU9150_DMP_CONFIG_SIZE);
        MPU_DEBUG_PRINTLN(F(" bytes in config def)"));
        if (writeProgDMPConfigurationSet(dmpConfig2, MPU9150_DMP_CONFIG_SIZE)) {
            MPU_DEBUG_PRINTLN(F("Success! DMP configuration written and verified."));

            MPU_DEBUG_PRINTLN(F("Setting DMP and FIFO_OFLOW interrupts enabled..."));
            setIntEnabled(0x12);

            MPU_DEBUG_PRINTLN(F("Setting sample rate to 200Hz..."));
            setRate(4); // 1khz / (1 + 4) = 200 Hz

            MPU_DEBUG_PRINTLN(F("Setting clock source to Z Gyro..."));
            setClockSource(MPU9150_CLOCK_PLL_ZGYRO);

            MPU_DEBUG_PRINTLN(F("Setting DLPF bandwidth to 42Hz..."));
            setDLPFMode(MPU9150_DLPF_BW_42);

            MPU_DEBUG_PRINTLN(F("Setting external frame sync to TEMP_OUT_L[0]..."));
            setExternalFrameSync(MPU9150_EXT_SYNC_TEMP_OUT_L);

            MPU_DEBUG_PRINTLN(F("Setting gyro sensitivity to +/- 2000 deg/sec..."));
            setFullScaleGyroRange(MPU9150_GYRO_FS_2000);

            MPU_DEBUG_PRINTLN(F("Setting DMP configuration bytes (function unknown)..."));
            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            MPU_DEBUG_PRINTLN(F("Clearing OTP Bank flag..."));
            setOTPBankValid(false);

            MPU_DEBUG_PRINTLN(F("Setting X/Y/Z gyro offsets to previous values..."));
            setXGyroOffsetTC(xgOffset);
            setYGyroOffsetTC(ygOffset);
            setZGyroOffsetTC(zgOffset);

            MPU_DEBUG_PRINTLN(F("Setting X/Y/Z gyro user offsets to zero..."));
            setXGyroOffset(0);
            setYGyroOffset(0);
            setZGyroOffset(0);

            MPU_DEBUG_PRINTLN(F("Writing final memory update 1/19 (function unknown)..."));
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Writing final memory update 2/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Resetting FIFO..."));
            resetFIFO();

            MPU_DEBUG_PRINTLN(F("Reading FIFO count..."));
            uint8_t fifoCount = getFIFOCount();

            MPU_DEBUG_PRINT(F("Current FIFO count="));
            MPU_DEBUG_PRINTLN(fifoCount);
            uint8_t fifoBuffer[128];
            //getFIFOBytes(fifoBuffer, fifoCount);

            MPU_DEBUG_PRINTLN(F("Writing final memory update 3/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Writing final memory update 4/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Disabling all standby flags..."));
            I2Cdev::writeByte(0x68, MPU9150_RA_PWR_MGMT_2, 0x00);

            MPU_DEBUG_PRINTLN(F("Setting accelerometer sensitivity to +/- 2g..."));
            I2Cdev::writeByte(0x68, MPU9150_RA_ACCEL_CONFIG, 0x00);

            MPU_DEBUG_PRINTLN(F("Setting motion detection threshold to 2..."));
            setMotionDetectionThreshold(2);

            MPU_DEBUG_PRINTLN(F("Setting zero-motion detection threshold to 156..."));
            setZeroMotionDetectionThreshold(156);

            MPU_DEBUG_PRINTLN(F("Setting motion detection duration to 80..."));
            setMotionDetectionDuration(80);

            MPU_DEBUG_PRINTLN(F("Setting zero-motion detection duration to 0..."));
            setZeroMotionDetectionDuration(0);

            MPU_DEBUG_PRINTLN(F("Setting AK8975 to single measurement mode..."));
            //mag -> setMode(1);
            I2Cdev::writeByte(0x0E, 0x0A, 0x01);

            // setup AK8975 (0x0E) as Slave 0 in read mode
            MPU_DEBUG_PRINTLN(F("Setting up AK8975 read slave 0..."));
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV0_ADDR, 0x8E);
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV0_REG,  0x01);
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV0_CTRL, 0xDA);

            // setup AK8975 (0x0E) as Slave 2 in write mode
            MPU_DEBUG_PRINTLN(F("Setting up AK8975 write slave 2..."));
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV2_ADDR, 0x0E);
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV2_REG,  0x0A);
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV2_CTRL, 0x81);
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV2_DO,   0x01);

            // setup I2C timing/delay control
            MPU_DEBUG_PRINTLN(F("Setting up slave access delay..."));
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_SLV4_CTRL, 0x18);
            I2Cdev::writeByte(0x68, MPU9150_RA_I2C_MST_DELAY_CTRL, 0x05);

            // enable interrupts
            MPU_DEBUG_PRINTLN(F("Enabling default interrupt behavior/no bypass..."));
            I2Cdev::writeByte(0x68, MPU9150_RA_INT_PIN_CFG, 0x00);

            // enable I2C master mode and reset DMP/FIFO
            // MPU_DEBUG_PRINTLN(F("Enabling I2C master mode..."));
            // I2Cdev::writeByte(0x68, MPU9150_RA_USER_CTRL, 0x20);
            // MPU_DEBUG_PRINTLN(F("Resetting FIFO..."));
            // I2Cdev::writeByte(0x68, MPU9150_RA_USER_CTRL, 0x24);
            // MPU_DEBUG_PRINTLN(F("Rewriting I2C master mode enabled because...I don't know"));
            // I2Cdev::writeByte(0x68, MPU9150_RA_USER_CTRL, 0x20);
            // MPU_DEBUG_PRINTLN(F("Enabling and resetting DMP/FIFO..."));
            // I2Cdev::writeByte(0x68, MPU9150_RA_USER_CTRL, 0xE8);
                  // setup weird slave stuff (?)
       
            MPU_DEBUG_PRINTLN(F("Writing final memory update 5/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 6/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 7/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 8/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 9/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 10/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 11/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            
            MPU_DEBUG_PRINTLN(F("Reading final memory update 12/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            #ifdef DEBUG
                MPU_DEBUG_PRINT(F("Read bytes: "));
                for (j = 0; j < 4; j++) {
                    MPU_DEBUG_PRINTF(dmpUpdate[3 + j], HEX);
                    MPU_DEBUG_PRINT(" ");
                }
                MPU_DEBUG_PRINTLN("");
            #endif

            MPU_DEBUG_PRINTLN(F("Writing final memory update 13/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 14/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 15/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 16/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);
            MPU_DEBUG_PRINTLN(F("Writing final memory update 17/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Waiting for FIRO count >= 46..."));
            while ((fifoCount = getFIFOCount()) < 46);
            MPU_DEBUG_PRINTLN(F("Reading FIFO..."));
            getFIFOBytes(fifoBuffer, (fifoCount < 128) ? fifoCount : 128); // safeguard only 128 bytes
            MPU_DEBUG_PRINTLN(F("Reading interrupt status..."));
            getIntStatus();

            MPU_DEBUG_PRINTLN(F("Writing final memory update 18/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Waiting for FIRO count >= 48..."));
            while ((fifoCount = getFIFOCount()) < 48);
            MPU_DEBUG_PRINTLN(F("Reading FIFO..."));
            getFIFOBytes(fifoBuffer, (fifoCount < 128) ? fifoCount : 128); // safeguard only 128 bytes
            MPU_DEBUG_PRINTLN(F("Reading interrupt status..."));
            getIntStatus();
            MPU_DEBUG_PRINTLN(F("Waiting for FIRO count >= 48..."));
            while ((fifoCount = getFIFOCount()) < 48);
            MPU_DEBUG_PRINTLN(F("Reading FIFO..."));
            getFIFOBytes(fifoBuffer, (fifoCount < 128) ? fifoCount : 128); // safeguard only 128 bytes
            MPU_DEBUG_PRINTLN(F("Reading interrupt status..."));
            getIntStatus();

            MPU_DEBUG_PRINTLN(F("Writing final memory update 19/19 (function unknown)..."));
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates2[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            MPU_DEBUG_PRINTLN(F("Disabling DMP (you turn it on later)..."));
            setDMPEnabled(false);

            MPU_DEBUG_PRINTLN(F("Setting up internal 48-byte (default) DMP packet buffer..."));
            dmpPacketSize = 48;
            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

            MPU_DEBUG_PRINTLN(F("Resetting FIFO and clearing INT status one last time..."));
            // resetFIFO();
            getIntStatus();
        } else {
            MPU_DEBUG_PRINTLN(F("ERROR! DMP configuration verification failed."));
            return 2; // configuration block loading failed
        }
    } else {
        MPU_DEBUG_PRINTLN(F("ERROR! DMP code verification failed."));
        return 1; // main binary block loading failed
    }
    return 0; // success
}


bool MPU6050_MPU9150::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

uint8_t MPU6050_MPU9150::dmpGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    if(!isMPU9150){
        data[0] = (packet[28] << 8) | packet[29];
        data[1] = (packet[32] << 8) | packet[33];
        data[2] = (packet[36] << 8) | packet[37];
    } else {
        data[0] = (packet[34] << 8) + packet[35];
        data[1] = (packet[38] << 8) + packet[39];
        data[2] = (packet[42] << 8) + packet[43];
    }
    return 0;
}
uint8_t MPU6050_MPU9150::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    if(!isMPU9150) {
        v -> x = (packet[28] << 8) | packet[29];
        v -> y = (packet[32] << 8) | packet[33];
        v -> z = (packet[36] << 8) | packet[37];
    } else {
        v -> x = (packet[34] << 8) + packet[35];
        v -> y = (packet[38] << 8) + packet[39];
        v -> z = (packet[42] << 8) + packet[43];
    }
    return 0;
}

uint8_t MPU6050_MPU9150::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) {
        packet = dmpPacketBuffer;
    }

        // if(isMPU9150) { 
            data[0] = ((packet[0] << 8) | packet[1]);
            data[1] = ((packet[4] << 8) | packet[5]);
            data[2] = ((packet[8] << 8) | packet[9]);
            data[3] = ((packet[12] << 8) | packet[13]); 
        // } else {
            // data[0] = ((packet[0] << 8) + packet[1]);
            // data[1] = ((packet[4] << 8) + packet[5]);
            // data[2] = ((packet[8] << 8) + packet[9]);
            // data[3] = ((packet[12] << 8) + packet[13]);
        // }
    return 0;
}
uint8_t MPU6050_MPU9150::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
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
uint8_t MPU6050_MPU9150::dmpGetGyro(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    // if(isMPU9150) {
        data[0] = (packet[16] << 8) | packet[17];
        data[1] = (packet[20] << 8) | packet[21];
        data[2] = (packet[24] << 8) | packet[25];
    // } else {
        // data[0] = (packet[16] << 8) + packet[17];
        // data[1] = (packet[20] << 8) + packet[21];
        // data[2] = (packet[24] << 8) + packet[25];
    // }
    return 0;
}
uint8_t MPU6050_MPU9150::dmpGetGyro(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    // if(isMPU9150) {
        v -> x = (packet[16] << 8) | packet[17];
        v -> y = (packet[20] << 8) | packet[21];
        v -> z = (packet[24] << 8) | packet[25];
    // } else {
        /*
        data[0] = (packet[28] << 8) + packet[29];
        data[1] = (packet[30] << 8) + packet[31];
        data[2] = (packet[32] << 8) + packet[33];
        */
       /*
        v -> x = (packet[28] << 8) | packet[29];
        v -> y = (packet[30] << 8) | packet[31];
        v -> z = (packet[32] << 8) | packet[33];
        */
        v -> x = (packet[16] << 8) | packet[17];
        v -> y = (packet[20] << 8) | packet[21];
        v -> z = (packet[24] << 8) | packet[25];
    // }
    return 0;
}
uint8_t MPU6050_MPU9150::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    if(isMPU9150) {
        v -> x = vRaw -> x - gravity -> x*8192;
        v -> y = vRaw -> y - gravity -> y*8192;
        v -> z = vRaw -> z - gravity -> z*8192;
    }else{
        v -> x = vRaw -> x - gravity -> x*4096;
        v -> y = vRaw -> y - gravity -> y*4096;
        v -> z = vRaw -> z - gravity -> z*4096;
    }
    return 0;
}
uint8_t MPU6050_MPU9150::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}
uint8_t MPU6050_MPU9150::dmpGetGravity(int16_t *data, const uint8_t* packet) {
    /* +1g corresponds to +8192, sensitivity is 2g. */
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
    data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
    data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
	       - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (2 * 16384);
    return status;
}

uint8_t MPU6050_MPU9150::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}
uint8_t MPU6050_MPU9150::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}

#ifdef USE_OLD_DMPGETYAWPITCHROLL
uint8_t MPU6050_MPU9150::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}
#else 
uint8_t MPU6050_MPU9150::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);
    if (gravity -> z < 0) {
        if(data[1] > 0) {
            data[1] = PI - data[1]; 
        } else { 
            data[1] = -PI - data[1];
        }
    }
    return 0;
}
#endif

uint8_t MPU6050_MPU9150::dmpProcessFIFOPacket(const unsigned char *dmpData) {
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t MPU6050_MPU9150::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;
        
        // increment external process count variable, if supplied
        if (processed != 0) (*processed)++;
    }
    return 0;
}

uint16_t MPU6050_MPU9150::dmpGetFIFOPacketSize() {
    return dmpPacketSize;
}

#endif /* _MPU6050_MPU9150_6AXIS_MOTIONAPPS20_H_ */
