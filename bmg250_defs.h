/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bmg250_defs.h
* @date 10/01/2020
* @version  1.1.2
*
*/

/*! @file bmg250_defs.h
 * @brief Sensor driver for BMG250 sensor
 */

/*!
 * @defgroup BMG250 SENSOR API
 * @brief
 * @{
 */
#ifndef BMG250_DEFS_H_
#define BMG250_DEFS_H_

/********************************************************/
/* header includes */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stdio.h>
#endif

#ifdef __KERNEL__
#if (LONG_MAX) > 0x7fffffff
#define __have_long64 1
#elif (LONG_MAX) == 0x7fffffff
#define __have_long32 1
#endif

#if !defined(UINT8_C)
#define INT8_C(x)   x
#if (INT_MAX) > 0x7f
#define UINT8_C(x)  x
#else
#define UINT8_C(x)  x##U
#endif
#endif

#if !defined(UINT16_C)
#define INT16_C(x)  x
#if (INT_MAX) > 0x7fff
#define UINT16_C(x) x
#else
#define UINT16_C(x) x##U
#endif
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#ifdef __have_long32
#define INT32_C(x)  x##L
#define UINT32_C(x) x##UL
#else
#define INT32_C(x)  x
#define UINT32_C(x) x##U
#endif
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#ifdef __have_long64
#define INT64_C(x)  x##L
#define UINT64_C(x) x##UL
#else
#define INT64_C(x)  x##LL
#define UINT64_C(x) x##ULL
#endif
#endif
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL                          0
#else
#define NULL                          ((void *) 0)
#endif
#endif

#define TRUE                          UINT8_C(1)
#define FALSE                         UINT8_C(0)

/**\name API success code */
#define BMG250_OK                     INT8_C(0)

/**\name API warning codes */
#define BMG250_W_SELF_TEST_FAIL       INT8_C(1)

/**\name API error codes */
#define BMG250_E_NULL_PTR             INT8_C(-1)
#define BMG250_E_COM_FAIL             INT8_C(-2)
#define BMG250_E_DEV_NOT_FOUND        INT8_C(-3)
#define BMG250_E_OUT_OF_RANGE         INT8_C(-4)
#define BMG250_E_INVALID_INPUT        INT8_C(-5)
#define BMG250_E_ODR_BW_INVALID       INT8_C(-6)
#define BMG250_E_INVALID_TEMPERATURE  INT8_C(-7)

/**\name BMG250 Settings selection macros */
#define BMG250_ODR_SEL                UINT8_C(0x01)
#define BMG250_BW_SEL                 UINT8_C(0x02)
#define BMG250_RANGE_SEL              UINT8_C(0x04)

/**\name Data select macros for reading sensor data */
#define BMG250_DATA_SEL               UINT8_C(0x01)
#define BMG250_DATA_TIME_SEL          UINT8_C(0x02)

/**\name ODR Settings */
#define BMG250_ODR_25HZ               UINT8_C(0x06)
#define BMG250_ODR_50HZ               UINT8_C(0x07)
#define BMG250_ODR_100HZ              UINT8_C(0x08)
#define BMG250_ODR_200HZ              UINT8_C(0x09)
#define BMG250_ODR_400HZ              UINT8_C(0x0A)
#define BMG250_ODR_800HZ              UINT8_C(0x0B)
#define BMG250_ODR_1600HZ             UINT8_C(0x0C)
#define BMG250_ODR_3200HZ             UINT8_C(0x0D)

/**\name BW Settings */
#define BMG250_BW_OSR4_MODE           UINT8_C(0x00)
#define BMG250_BW_OSR2_MODE           UINT8_C(0x01)
#define BMG250_BW_NORMAL_MODE         UINT8_C(0x02)

/**\name Range Settings */
#define BMG250_RANGE_2000_DPS         UINT8_C(0x00)
#define BMG250_RANGE_1000_DPS         UINT8_C(0x01)
#define BMG250_RANGE_500_DPS          UINT8_C(0x02)
#define BMG250_RANGE_250_DPS          UINT8_C(0x03)
#define BMG250_RANGE_125_DPS          UINT8_C(0x04)

/**\name Interface settings */
#define BMG250_SPI_RD_MASK            UINT8_C(0x80)
#define BMG250_SPI_WR_MASK            UINT8_C(0x7F)

/**\name Interface settings */
#define BMG250_SPI_COMM_TEST_ADDR     UINT8_C(0x7F)

/**\name BMG250 Chip ID */
#define BMG250_CHIP_ID                UINT8_C(0XD5)

/**\name BMG250 I2C address */
/**\name I2C address when SDO pulled to GND */
#define BMG250_I2C_ADDR               UINT8_C(0x68)

/**\name address when SDO pulled to VDDIO */
#define BMG250_SEC_I2C_ADDR           UINT8_C(0x69)

/**\name BMG250 Register map */
#define BMG250_CHIP_ID_ADDR           UINT8_C(0x00)
#define BMG250_ERROR_REG_ADDR         UINT8_C(0x02)
#define BMG250_PMU_STATUS_ADDR        UINT8_C(0x03)
#define BMG250_DATA_ADDR              UINT8_C(0x12)
#define BMG250_STATUS_ADDR            UINT8_C(0x1B)
#define BMG250_INT_STATUS_ADDR        UINT8_C(0x1D)
#define BMG250_TEMPERATURE_ADDR       UINT8_C(0x20)
#define BMG250_FIFO_LENGTH_ADDR       UINT8_C(0x22)
#define BMG250_FIFO_DATA_ADDR         UINT8_C(0x24)
#define BMG250_GYRO_CONFIG_ADDR       UINT8_C(0x42)
#define BMG250_GYRO_RANGE_ADDR        UINT8_C(0x43)
#define BMG250_FIFO_DOWN_ADDR         UINT8_C(0x45)
#define BMG250_FIFO_CONFIG0_ADDR      UINT8_C(0x46)
#define BMG250_FIFO_CONFIG1_ADDR      UINT8_C(0x47)
#define BMG250_INT_EN_ADDR            UINT8_C(0x51)
#define BMG250_IN_OUT_CNTRL_ADDR      UINT8_C(0x53)
#define BMG250_INT_IN_CTRL_ADDR       UINT8_C(0x54)
#define BMG250_INT_MAP_ADDR           UINT8_C(0x56)
#define BMG250_IF_CONF_ADDR           UINT8_C(0x6B)
#define BMG250_SELF_TEST_ADDR         UINT8_C(0x6D)
#define BMG250_NV_CONF_ADDR           UINT8_C(0x70)
#define BMG250_OFFSET_X_ADDR          UINT8_C(0x74)
#define BMG250_OFFSET_Y_ADDR          UINT8_C(0x75)
#define BMG250_OFFSET_Z_ADDR          UINT8_C(0x76)
#define BMG250_OFFSET_EN_ADDR         UINT8_C(0x77)
#define BMG250_COMMAND_REG_ADDR       UINT8_C(0x7E)

/**\name BMG250 Command register */
#define BMG250_FIFO_FLUSH_CMD         UINT8_C(0xB0)
#define BMG250_SOFT_RESET_CMD         UINT8_C(0xB6)
#define BMG250_FOC_ENABLE_CMD         UINT8_C(0x03)

/**\name BMG250 Self test trigger */
#define BMG250_SELF_TEST_TRIGGER      UINT8_C(0x10)
#define BMG250_SELF_TEST_SUCCESS      UINT8_C(0x01)
#define BMG250_SELF_TEST_FAIL         UINT8_C(0x00)

/**\name BMG250 Delay definitions */
#define BMG250_GYRO_DELAY_MS          UINT8_C(81)
#define BMG250_SOFT_RESET_DELAY_MS    UINT8_C(15)
#define BMG250_GYRO_FSU_DELAY_MS      UINT8_C(10)
#define BMG250_FOC_DELAY_MS           UINT8_C(500)

/**\name Gyro power mode */
#define BMG250_GYRO_SUSPEND_MODE      UINT8_C(0x14)
#define BMG250_GYRO_NORMAL_MODE       UINT8_C(0x15)
#define BMG250_GYRO_FASTSTARTUP_MODE  UINT8_C(0x17)

/**\name Gyro power mode status macros */
#define BMG250_PMU_STATUS_SUSPEND     UINT8_C(0x00)
#define BMG250_PMU_STATUS_FSU         UINT8_C(0x03)

/**\name Maximum limits definition */
#define BMG250_GYRO_ODR_MAX           UINT8_C(13)
#define BMG250_GYRO_BW_MAX            UINT8_C(2)
#define BMG250_GYRO_RANGE_MAX         UINT8_C(4)

/**\name Mask definitions for FIFO_CONFIG register */
#define BMG250_FIFO_GYRO              UINT8_C(0x80)
#define BMG250_FIFO_HEADER            UINT8_C(0x10)
#define BMG250_FIFO_TAG_INT1          UINT8_C(0x08)
#define BMG250_FIFO_TAG_INT2          UINT8_C(0x04)
#define BMG250_FIFO_TIME              UINT8_C(0x02)
#define BMG250_FIFO_STOP_ON_FULL      UINT8_C(0x01)
#define BMG250_FIFO_ALL_SETTING       UINT8_C(0x9F)

/**\name BMG250 FIFO down sampling Settings */
#define BMG250_GYRO_FIFO_FILT_EN      UINT8_C(0x08)
#define BMG250_GYRO_FIFO_DOWN_ZERO    UINT8_C(0x00)
#define BMG250_GYRO_FIFO_DOWN_ONE     UINT8_C(0x01)
#define BMG250_GYRO_FIFO_DOWN_TWO     UINT8_C(0x02)
#define BMG250_GYRO_FIFO_DOWN_THREE   UINT8_C(0x03)
#define BMG250_GYRO_FIFO_DOWN_FOUR    UINT8_C(0x04)
#define BMG250_GYRO_FIFO_DOWN_FIVE    UINT8_C(0x05)
#define BMG250_GYRO_FIFO_DOWN_SIX     UINT8_C(0x06)
#define BMG250_GYRO_FIFO_DOWN_SEVEN   UINT8_C(0x07)

/**\name BMG250 FIFO Definitions */
#define BMG250_SENSOR_TIME_LENGTH     UINT8_C(3)
#define BMG250_FIFO_G_HEADER_LENGTH   UINT8_C(7)
#define BMG250_FIFO_G_LENGTH          UINT8_C(6)
#define BMG250_FIFO_BYTE_COUNTER_MASK UINT8_C(0x07)

/** Mask definition for FIFO Header Data Tag */
#define BMG250_FIFO_TAG_INTR_MASK     UINT8_C(0xFC)

/** FIFO Header Data definitions */
#define BMG250_FIFO_HEAD_G            UINT8_C(0x88)
#define BMG250_FIFO_HEAD_SKIP_FRAME   UINT8_C(0x40)
#define BMG250_FIFO_HEAD_SENSOR_TIME  UINT8_C(0x44)
#define BMG250_FIFO_HEAD_INPUT_CONFIG UINT8_C(0x48)
#define BMG250_FIFO_HEAD_OVER_READ    UINT8_C(0x80)

/** Definitions to check validity of FIFO frames */
#define FIFO_CONFIG_MSB_CHECK         UINT8_C(0x80)
#define FIFO_CONFIG_LSB_CHECK         UINT8_C(0x00)

/**\name BMG250 interrupt setting macros */
#define BMG250_DISABLE                UINT8_C(0)
#define BMG250_ENABLE                 UINT8_C(1)
#define BMG250_PUSH_PULL              UINT8_C(0)
#define BMG250_OPEN_DRAIN             UINT8_C(1)
#define BMG250_ACTIVE_LOW             UINT8_C(0)
#define BMG250_ACTIVE_HIGH            UINT8_C(1)
#define BMG250_LEVEL_TRIGGER          UINT8_C(0)
#define BMG250_EDGE_TRIGGER           UINT8_C(1)

/**\name BMG250 interrupt status macros */
#define DRDY_INT_ASSERTED             UINT8_C(0x10)
#define FIFO_FULL_INT_ASSERTED        UINT8_C(0x20)
#define FIFO_WM_INT_ASSERTED          UINT8_C(0x40)

/**\name BMG250 Status macros */
#define BMG250_FOC_READY              UINT8_C(0x08)

/**\name BMG250 if_mode configurations */
#define BMG250_IF_AUTO_CONFIG         UINT8_C(0)
#define BMG250_IF_I2C_OIS             UINT8_C(1)

/**\name BMG250 Watch-dog timer configurations */
#define BMG250_I2C_WDT_1_MS           UINT8_C(0)
#define BMG250_I2C_WDT_50_MS          UINT8_C(1)

/**\name BMG250 position and mask macros */
#define BMG250_GYRO_ODR_MSK           UINT8_C(0x0F)

#define BMG250_GYRO_BW_MSK            UINT8_C(0x30)
#define BMG250_GYRO_BW_POS            UINT8_C(4)

#define BMG250_GYRO_RANGE_MSK         UINT8_C(0x07)
#define BMG250_ERR_REG_MASK           UINT8_C(0x0F)

#define BMG250_GYRO_PMU_STATUS_MSK    UINT8_C(0x0C)
#define BMG250_GYRO_PMU_STATUS_POS    UINT8_C(2)

#define BMG250_DATA_READY_EN_MSK      UINT8_C(0x10)
#define BMG250_DATA_READY_EN_POS      UINT8_C(4)

#define BMG250_FIFO_FULL_EN_MSK       UINT8_C(0x20)
#define BMG250_FIFO_FULL_EN_POS       UINT8_C(5)

#define BMG250_FIFO_WM_EN_MSK         UINT8_C(0x40)
#define BMG250_FIFO_WM_EN_POS         UINT8_C(6)

#define BMG250_INT1_EN_MSK            UINT8_C(0x08)
#define BMG250_INT1_EN_POS            UINT8_C(3)

#define BMG250_INT1_OD_MSK            UINT8_C(0x04)
#define BMG250_INT1_OD_POS            UINT8_C(2)

#define BMG250_INT1_LVL_MSK           UINT8_C(0x02)
#define BMG250_INT1_LVL_POS           UINT8_C(1)

#define BMG250_INT1_EDGE_MSK          UINT8_C(0x01)

#define BMG250_INT2_EN_MSK            UINT8_C(0x80)
#define BMG250_INT2_EN_POS            UINT8_C(7)

#define BMG250_INT2_OD_MSK            UINT8_C(0x40)
#define BMG250_INT2_OD_POS            UINT8_C(6)

#define BMG250_INT2_LVL_MSK           UINT8_C(0x20)
#define BMG250_INT2_LVL_POS           UINT8_C(5)

#define BMG250_INT2_EDGE_MSK          UINT8_C(0x10)
#define BMG250_INT2_EDGE_POS          UINT8_C(4)

#define BMG250_INT1_INPUT_EN_MSK      UINT8_C(0x10)
#define BMG250_INT1_INPUT_EN_POS      UINT8_C(4)

#define BMG250_INT2_INPUT_EN_MSK      UINT8_C(0x20)
#define BMG250_INT2_INPUT_EN_POS      UINT8_C(5)

#define BMG250_INT1_DATA_READY_MSK    UINT8_C(0x80)
#define BMG250_INT1_DATA_READY_POS    UINT8_C(7)

#define BMG250_INT2_DATA_READY_MSK    UINT8_C(0x08)
#define BMG250_INT2_DATA_READY_POS    UINT8_C(3)

#define BMG250_INT1_FIFO_FULL_MSK     UINT8_C(0x20)
#define BMG250_INT1_FIFO_FULL_POS     UINT8_C(5)

#define BMG250_INT2_FIFO_FULL_MSK     UINT8_C(0x02)
#define BMG250_INT2_FIFO_FULL_POS     UINT8_C(1)

#define BMG250_INT1_FIFO_WM_MSK       UINT8_C(0x40)
#define BMG250_INT1_FIFO_WM_POS       UINT8_C(6)

#define BMG250_INT2_FIFO_WM_MSK       UINT8_C(0x04)
#define BMG250_INT2_FIFO_WM_POS       UINT8_C(2)

#define BMG250_FIFO_HEADER_EN_MSK     UINT8_C(0x10)
#define BMG250_FIFO_HEADER_EN_POS     UINT8_C(4)

#define BMG250_FIFO_TIME_EN_MSK       UINT8_C(0x02)
#define BMG250_FIFO_TIME_EN_POS       UINT8_C(1)

#define BMG250_FIFO_DATA_EN_MSK       UINT8_C(0x80)
#define BMG250_FIFO_DATA_EN_POS       UINT8_C(7)

#define BMG250_SELF_TEST_RSLT_MSK     UINT8_C(0x02)
#define BMG250_SELF_TEST_RSLT_POS     UINT8_C(1)

#define BMG250_OFFSET_ENABLE_MSK      UINT8_C(0x80)
#define BMG250_OFFSET_ENABLE_POS      UINT8_C(7)

#define BMG250_GYRO_OFFSET_MSB_MSK    UINT16_C(0x0300)
#define BMG250_GYRO_OFFSET_MSB_POS    UINT16_C(8)

#define BMG250_GYRO_OFFSET_LSB_MSK    UINT16_C(0x00FF)

#define BMG250_FOC_READY_MSK          UINT8_C(0x08)
#define BMG250_FOC_READY_POS          UINT8_C(3)

#define BMG250_WDT_EN_MSK             UINT8_C(0x04)
#define BMG250_WDT_EN_POS             UINT8_C(2)

#define BMG250_WDT_SEL_MSK            UINT8_C(0x02)
#define BMG250_WDT_SEL_POS            UINT8_C(1)

#define BMG250_IF_CONFIG_MSK          UINT8_C(0x10)
#define BMG250_IF_CONFIG_POS          UINT8_C(4)

#define BMG250_SPI_EN_MSK             UINT8_C(0x01)

#define BMG250_FIFO_DOWN_SAMPLING_MSK UINT8_C(0x0F)

/**\name BIT SLICE GET AND SET FUNCTIONS */
#define BMG250_GET_BITS(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)
#define BMG250_SET_BITS(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | \
     ((val << bitname##_POS) & bitname##_MSK))

#define BMG250_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMG250_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

/********************************************************/

/*!
 * @brief Interface selection Enums
 */
enum bmg250_intf {
    /*! SPI interface */
    BMG250_SPI_INTF,

    /*! I2C interface */
    BMG250_I2C_INTF
};

/********************************************************/

/*!
 * @brief Type definitions
 */
typedef int8_t (*bmg250_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bmg250_delay_fptr_t)(uint32_t period);

/********************************************************/

/*!
 * @brief bmg250 sensor configuration structure
 */
struct bmg250_cfg
{
    /*! output data rate */
    uint8_t odr;

    /*! range */
    uint8_t range;

    /*! bandwidth */
    uint8_t bw;
};

/*!
 * @brief bmg250 sensor data structure which has gyro data
 */
struct bmg250_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;

    /*! sensor time */
    uint32_t sensortime;
};

/*!
 * @brief bmg250 interrupt channel selection enum
 */
enum bmg250_int_channel {
    /*! interrupt Channel 1 */
    BMG250_INT_CHANNEL_1,

    /*! interrupt Channel 2 */
    BMG250_INT_CHANNEL_2
};

/*!
 * @brief bmg250 interrupt type selection enum
 */
enum bmg250_int_types {
    /*! data ready interrupt  */
    BMG250_DATA_RDY_INT,

    /*! fifo full interrupt */
    BMG250_FIFO_FULL_INT,

    /*! fifo watermark interrupt */
    BMG250_FIFO_WATERMARK_INT
};

/*!
 * @brief bmg250 interrupt pin setting
 */
struct bmg250_int_pin_settg
{
    /*! To enable either INT1 or INT2 pin as output.
     * 0- output disabled ,1- output enabled
     */
    uint8_t output_en;

    /*! 0 - push-pull 1- open drain,only valid if output_en is set 1 */
    uint8_t output_mode;

    /*! 0 - active low , 1 - active high level.
     * if output_en is 1,this applies to interrupts,else PMU_trigger
     */
    uint8_t output_type;

    /*! 0 - level trigger , 1 - edge trigger  */
    uint8_t edge_ctrl;

    /*! To enable either INT1 or INT2 pin as input.
     * 0 - input disabled ,1 - input enabled
     */
    uint8_t input_en;
};

/*!
 * @brief bmg250 interrupt setting
 */
struct bmg250_int_settg
{
    /*! Interrupt channel */
    enum bmg250_int_channel int_channel;

    /*! Select Interrupt */
    enum bmg250_int_types int_type;

    /*! Structure configuring Interrupt pins */
    struct bmg250_int_pin_settg int_pin_settg;

    /*! FIFO FULL INT 1-enable, 0-disable */
    uint8_t fifo_full_int_en;

    /*! FIFO WTM INT 1-enable, 0-disable */
    uint8_t fifo_wtm_int_en;
};

/*!
 *  @brief This structure holds the information for usage of
 *  FIFO by the user.
 */
struct bmg250_fifo_frame
{
    /*! Data buffer of user defined length is to be mapped here */
    uint8_t *data;

    /*! While calling the API  "bmg250_get_fifo_data" , length stores
     *  number of bytes in FIFO to be read (specified by user as input)
     *  and after execution of the API ,number of FIFO data bytes
     *  available is provided as an output to user
     */
    uint16_t length;

    /*! FIFO time enable */
    uint8_t fifo_time_enable;

    /*! Enabling of the FIFO header to stream in header mode */
    uint8_t fifo_header_enable;

    /*! Streaming of the data in FIFO */
    uint8_t fifo_data_enable;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t gyro_byte_start_idx;

    /*! Value of FIFO sensor time time */
    uint32_t sensor_time;

    /*! Value of Skipped frame counts */
    uint8_t skipped_frame_count;

    /*! Value to select FIFO downsampling and
     *  filtered/unfiltered data in FIFO
     */
    uint8_t fifo_down;
};

/*!
 * @brief bmg250 offset structure
 */
struct bmg250_offset
{
    /*XYZ offset */
    uint16_t x_offset;
    uint16_t y_offset;
    uint16_t z_offset;
};

/*!
 * @brief bmg250 device structure
 */
struct bmg250_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! Device Id */
    uint8_t dev_id;

    /*! SPI/I2C interface */
    enum bmg250_intf intf;

    /*! Read function pointer */
    bmg250_com_fptr_t read;

    /*! Write function pointer */
    bmg250_com_fptr_t write;

    /*! Delay function pointer */
    bmg250_delay_fptr_t delay_ms;

    /*! FIFO related configurations */
    struct bmg250_fifo_frame *fifo;

    /* Power mode of sensor */
    uint8_t power_mode;
};

#endif /* BMG250_DEFS_H_ */
/** @}*/
/** @}*/
