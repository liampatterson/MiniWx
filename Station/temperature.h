#include <fsl_debug_console.h>
#include <board.h>
#include "utils.h"

#define DATA_LENGTH 64

#include "Driver_I2C.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include <string.h>

    //misc inits
    #define SENSOR_ADDRESS                (0x77)
		#define PRINTF_FLOAT_ENABLE   1

    //SENSOR REG ADDRESSES, From Bosch Datasheet
    enum
    {
        SENSOR_CHIPID             = 0xD0, //1 call
        SENSOR_READVAL            = 0x60,
        SENSOR_SOFTRESET          = 0xE0, //2 call
        SENSOR_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0
        SENSOR_CONTROLHUMID       = 0xF2,
        SENSOR_STATUS             = 0xF3, //3 call
        SENSOR_CONTROL            = 0xF4,
        SENSOR_CONFIG             = 0xF5,
        SENSOR_PRESSUREDATA       = 0xF7,
        SENSOR_TEMPDATA           = 0xFA,
        SENSOR_HUMIDDATA          = 0xFD,

        SENSOR_DIG_T1              = 0x88,
        SENSOR_DIG_T2              = 0x8A,
        SENSOR_DIG_T3              = 0x8C,

        SENSOR_DIG_P1              = 0x8E,
        SENSOR_DIG_P2              = 0x90,
        SENSOR_DIG_P3              = 0x92,
        SENSOR_DIG_P4              = 0x94,
        SENSOR_DIG_P5              = 0x96,
        SENSOR_DIG_P6              = 0x98,
        SENSOR_DIG_P7              = 0x9A,
        SENSOR_DIG_P8              = 0x9C,
        SENSOR_DIG_P9              = 0x9E,

        SENSOR_DIG_H1              = 0xA1,
        SENSOR_DIG_H2              = 0xE1,
        SENSOR_DIG_H3              = 0xE3,
        SENSOR_DIG_H4              = 0xE4,
        SENSOR_DIG_H5              = 0xE5,
        SENSOR_DIG_H6              = 0xE7
    };

//make sure the variables are declared for the regs above

		uint16_t    dig_T1;
    int16_t     dig_T2, dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint16_t    dig_H1, dig_H3;
    int16_t     dig_H2, dig_H4, dig_H5, dig_H6;
    int32_t     t_fine;
