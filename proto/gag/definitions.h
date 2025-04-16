/**
 * 
*/
#define ESP32_RIGHT 1
#define SEND_ACC
#define MEASURE_OFFSETS 1
#define SET_OFFSETS 1
// #define SEND_DATA_ALSO_OVER_SERIAL 1

#ifdef ESP32_RIGHT
#define MASTER_HAND
// #define USE_DISPLAY 1
#define MASTER_BT_SERIAL
#define USE_BT_GATT_SERIAL
#else
#define SLAVE_HAND
#endif

#define MEASUREMENT_LIMIT 40

#define GAG_DEBUG
#ifdef GAG_DEBUG
#define GAG_DEBUG_PRINT(x) Serial.print(x)
#define GAG_DEBUG_PRINTF(x, y) Serial.print(x, y)
#define GAG_DEBUG_PRINTLN(x) Serial.println(x)
#define GAG_DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
#define GAG_DEBUG_PRINT(x)
#define GAG_DEBUG_PRINTF(x, y)
#define GAG_DEBUG_PRINTLN(x)
#define GAG_DEBUG_PRINTLNF(x, y)
#endif

#define MPU9150_DEBUG
#ifdef MPU9150_DEBUG
#define MPU9150_DEBUG_PRINT(x) Serial.print(x)
#define MPU9150_DEBUG_PRINTF(x, y) Serial.print(x, y)
#define MPU9150_DEBUG_PRINTLN(x) Serial.println(x)
#define MPU9150_DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
#define GAG_DEBUG_PRINT(x)
#define GAG_DEBUG_PRINTF(x, y)
#define GAG_DEBUG_PRINTLN(x)
#define GAG_DEBUG_PRINTLNF(x, y)
#endif


#define MPU6050_FIFO_PACKET_SIZE 42
#define MPU9150_FIFO_PACKET_SIZE 48
// TODO fix..
#define FIFO_SIZE 48


#ifndef ESP32_RIGHT
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
#endif
#endif

#define CMD_TEST_REPLY 't'
#define CMD_CALIBRATION 'a'
#define CMD_SET_SENSOR_DEBUG_FLAG 'd'
#define CMD_RESTART_WITH_CALIBRATION_AND_SEND 's'
#define CMD_RESTART 'r'
#define CMD_READ_SLAVE 'i'
#define CMD_DONT_READ_SLAVE 'I'
#define CMD_GET_CURRENT_OFFSET 'o'
#define CMD_SET_OFFSET 'O'

#define CMD_PACKET_LENGTH 9
#ifdef SEND_ACC
#define PACKET_LENGTH 21
#define PACKET_COUNTER_POSITION 18
#else
#define PACKET_LENGTH 15
#define PACKET_COUNTER_POSITION 10
#endif
#ifdef ESP32_RIGHT

#define SENSORS_COUNT 7
// SENSORs <0,5>
#define FIRST_SENSOR 0
// or set LAST_SENSOR to 5
#define LAST_SENSOR 6
// time for internal interrupt to trigger in loop - working up to 50 ms but freezes may occure - so reset MPU's FIFO more ften (20ms each?)
#define SWITCH_SENSORS_MS 0

// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

#define SENSOR_PIN_TU TU
#define SENSOR_PIN_TU_COMPENSATION 14
#define SENSOR_PIN_SU SU
#define SENSOR_PIN_SU_COMPENSATION 13
#define SENSOR_PIN_FU FU
#define SENSOR_PIN_FU_COMPENSATION 12
#define SENSOR_PIN_MU MU
#define SENSOR_PIN_MU_COMPENSATION 4
#define SENSOR_PIN_EU EU
#define SENSOR_PIN_EU_COMPENSATION 2
#define SENSOR_PIN_HP HP  // HAND PALM
#define SENSOR_PIN_HP_COMPENSATION 15
#define SENSOR_PIN_HG HG  // HAND PALM MPU6050
#define SENSOR_PIN_HG_COMPENSATION 23
#define SENSOR_PIN_NF NF

//#define SENSOR_PIN_OFFSET 3
#define SENSOR_PIN_OFFSET 0
// 57600 115200
//#define BT_BAUD 57600

#else
#define SENSORS_COUNT 6
#define FIRST_SENSOR 0
#define LAST_SENSOR 5
// time for internal interrupt to trigger in loop - working up to 50 ms but freezes may occure - so reset MPU's FIFO more ften (20ms each?)
#define SWITCH_SENSORS_MS 0

#define SENSOR_PIN_TU TU
#define SENSOR_PIN_TU_COMPENSATION 4
#define SENSOR_PIN_SU SU
#define SENSOR_PIN_SU_COMPENSATION 7
#define SENSOR_PIN_FU FU
#define SENSOR_PIN_FU_COMPENSATION 10
#define SENSOR_PIN_MU MU
#define SENSOR_PIN_MU_COMPENSATION 11
#define SENSOR_PIN_EU EU
#define SENSOR_PIN_EU_COMPENSATION 12
#define SENSOR_PIN_HP HP  // HAND PALM
#define SENSOR_PIN_HP_COMPENSATION 13
#define SENSOR_PIN_NF NF

#define SENSOR_PIN_OFFSET 0
// 57600 115200
#endif

// TODO do not use this at all
#define MAX_HAND_SWITCH_TIME 25
// do not forget for ShanonKotelnik theorem where MAX_HAND_SWITCH_CHARS >= PACKET_LENGTH*2
#define MAX_HAND_SWITCH_CHARS 35
// TODO make left/right switch

// TODO test if value 2 is enough
#define REPEAT_MASTER_HAND_READ_LIMIT 10
#define REPEAT_SLAVE_HAND_READ_LIMIT 10

// (9600 38400 57600 74880 115200 230400 250000 57600 38400 chosen because it is required for output, but it's

#ifdef SLAVE_HAND
#define MAX_TIME_TO_RESET 80
#define MIN_TIME_TO_RESET 20
#define MAX_FIFO_USAGE_FOR_RESET 300
#define MIN_FIFO_USAGE_FOR_RESET 150
//#define FIFO_PACKET_SIZE FIFO_APCKET_GLOBAL_SIZE
//#define FIFO_SIZE FIFO_APCKET_GLOBAL_SIZE
#else
#ifdef MASTER_HAND
// values for ESP32 400KHz I2C
// comparing to nano MPU6050 needs to be reset more often
#define MAX_TIME_TO_RESET 15
#define MIN_TIME_TO_RESET 5
#define MAX_FIFO_USAGE_FOR_RESET 100
#define MIN_FIFO_USAGE_FOR_RESET 50
#else
// values for arduino nano using 400KHz I2C
#define MAX_TIME_TO_RESET 50
#define MIN_TIME_TO_RESET 20
#define MAX_FIFO_USAGE_FOR_RESET 300
#define MIN_FIFO_USAGE_FOR_RESET 200
#endif
#endif