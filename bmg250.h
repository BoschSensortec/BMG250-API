/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bmg250.h
 * @date    Jun 21 2017
 * @version 1.0.0
 * @brief   Sensor driver for BMG250 sensor
 *
 */

#ifndef BMG250_H_
#define BMG250_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C"
{
#endif

/* Header includes */
#include "bmg250_defs.h"

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of bmg250 sensor.
 *
 * @param[in] dev       : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_init(struct bmg250_dev *dev);

/*!
 * @brief This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] data     : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in] dev       : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmg250_dev *dev);

/*!
 * @brief This API writes the given data to the register address of sensor.
 *
 * @param[in] reg_addr  : Register address where the data is to be written.
 * @param[in] data      : Pointer to data buffer, whose data is to be written
 *                        in the sensor.
 * @param[in] len       : No of bytes of data to be writen.
 * @param[in] dev       : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmg250_dev *dev);

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 *
 * @param[in] dev       : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_soft_reset(const struct bmg250_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev     : Structure instance of bmg250_dev.
 *
 * @note prerequisite : Set the required macro to dev->gyro_cfg.power
 * set the corresponding power mode from the below table
 *
 *       dev->power_mode        |  Power mode set
 * -----------------------------|----------------------
 * BMG250_GYRO_SUSPEND_MODE     |   Suspend mode
 * BMG250_GYRO_NORMAL_MODE      |   Normal mode
 * BMG250_GYRO_FASTSTARTUP_MODE |   Fast-startup mode
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_power_mode(const struct bmg250_dev *dev);

/*!
 * @brief This API sets the sensor configurations like ODR, Range, BW.
 *
 * @param[in] gyro_cfg          : Structure instance to configure gyro
 * @param[in] dev               : Structure instance of bmg250_dev.
 *
 * @note prerequisite : Call the API "bmg250_get_sensor_settings" to get
 * the default settings in the sensor
 *
 * @note prerequisite : Set the required macro to gyro_cfg to
 * set the corresponding sensor configurations from the below table
 *
 *    gyro_cfg->odr    |    gyro_cfg->range    |    gyro_cfg->bw
 * --------------------|-----------------------|-----------------------
 * BMG250_ODR_25HZ     | BMG250_RANGE_2000_DPS |  BMG250_BW_OSR4_MODE
 * BMG250_ODR_50HZ     | BMG250_RANGE_1000_DPS |  BMG250_BW_OSR2_MODE
 * BMG250_ODR_100HZ    | BMG250_RANGE_500_DPS  |  BMG250_BW_NORMAL_MODE
 * BMG250_ODR_200HZ    | BMG250_RANGE_250_DPS  |         -
 * BMG250_ODR_400HZ    | BMG250_RANGE_125_DPS  |         -
 * BMG250_ODR_800HZ    |         -             |         -
 * BMG250_ODR_1600HZ   |         -             |         -
 * BMG250_ODR_3200HZ   |         -             |         -
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_sensor_settings(const struct bmg250_cfg *gyro_cfg, const struct bmg250_dev *dev);

/*!
 * @brief This API gets the sensor configurations like
 * ODR, Range, BW from the sensor
 *
 * @param[in] gyro_cfg          : Structure instance to configure gyro
 * @param[in] dev               : Structure instance of bmg250_dev.
 *
 * @note prerequisite : Sensor setting can be obtained from the
 * gyro_cfg structure after calling the "bmg250_get_sensor_settings"
 *
 *    gyro_cfg->odr    |    gyro_cfg->range    |    gyro_cfg->bw
 * --------------------|-----------------------|-----------------------
 * BMG250_ODR_25HZ     | BMG250_RANGE_2000_DPS |  BMG250_BW_OSR4_MODE
 * BMG250_ODR_50HZ     | BMG250_RANGE_1000_DPS |  BMG250_BW_OSR2_MODE
 * BMG250_ODR_100HZ    | BMG250_RANGE_500_DPS  |  BMG250_BW_NORMAL_MODE
 * BMG250_ODR_200HZ    | BMG250_RANGE_250_DPS  |         -
 * BMG250_ODR_400HZ    | BMG250_RANGE_125_DPS  |         -
 * BMG250_ODR_800HZ    |         -             |         -
 * BMG250_ODR_1600HZ   |         -             |         -
 * BMG250_ODR_3200HZ   |         -             |         -
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_get_sensor_settings(struct bmg250_cfg *gyro_cfg, const struct bmg250_dev *dev);

/*!
 * @brief This API reads gyro data along with sensor time if time is requested
 * by user. Kindly refer the user guide(README.md) for more info.
 *
 * @param[in] data_sel    : Selection macro to select data/data & sensor-time
 * @param[in] gyro        : Structure pointer to store gyro data
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 *  Value of "data_sel"   |  Data available in "gyro"
 * -----------------------|----------------------------
 * BMG250_DATA_SEL        |   Gyro data
 * BMG250_DATA_TIME_SEL   |   Gyro data + sensortime
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_get_sensor_data(uint8_t data_sel, struct bmg250_sensor_data *gyro, const struct bmg250_dev *dev);

/*!
 * @brief This API configures the necessary interrupt based on
 * the user settings in the bmg250_int_settg structure instance.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 * @note : Refer user guide for detailed info.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_int_config(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This API gives us the status of the interrupts
 * whether they are triggered or not
 *
 * @param[out] int_status  : interrupt status information.
 * @param[in] dev          : Structure instance of bmg250_dev.
 *
 * @note int_status can be "ANDed" with the following macros to
 * detect which interrupt has been asserted
 *   - DRDY_INT_ASSERTED
 *   - FIFO_FULL_INT_ASSERTED
 *   - FIFO_WM_INT_ASSERTED
 *
 * Ex. if (int_status & DRDY_INT_ASSERTED) {
 *		printf("data ready interrupt asserted");
 *     } else {
 *		printf("data ready interrupt not asserted");
 *     }
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_get_int_status(uint8_t *int_status, const struct bmg250_dev *dev);

/*! @brief This API sets the FIFO configuration in the sensor.
 *
 *  @param[in] config : variable used to specify the FIFO
 *  configurations which are to be enabled or disabled in the sensor.
 *
 *  @note : User can set either set one or more or all FIFO configurations
 *  by ORing the below mentioned macros and assigning them to "config".
 *    Possible value for parameter "config" :
 *      - BMG250_FIFO_GYRO
 *      - BMG250_FIFO_HEADER
 *      - BMG250_FIFO_TAG_INT1
 *      - BMG250_FIFO_TAG_INT2
 *      - BMG250_FIFO_TIME
 *      - BMG250_FIFO_STOP_ON_FULL
 *
 *  @param[in] enable : Parameter used to enable or disable the above
 *                      FIFO configuration
 *     Possible value for parameter "enable" :
 *      - BMG250_DISABLE
 *      - BMG250_ENABLE
 *
 *  @param[in] dev : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_fifo_config(uint8_t config, uint8_t enable, const struct bmg250_dev *dev);

/*!
 *  @brief This API writes fifo_flush command to command register.
 *  This action clears all data in the sensor's FIFO without changing
 *  FIFO configuration settings
 *
 *  @param[in] dev : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_fifo_flush(const struct bmg250_dev *dev);

/*!
 *  @brief This API is used to configure the down sampling ratios
 *  for sensor data in FIFO.
 *  It also configures either filtered/pre-filtered data to
 *  be available in FIFO
 *
 *  @param[in] fifo_down : variable used to specify the FIFO down
 *  configurations which are to be enabled or disabled in the sensor.
 *
 *  @note The user can enable filtered gyro data by using the following macro
 *      fifo_down                  |   Value
 *    -----------------------------|---------------------------
 *      BMG250_GYRO_FIFO_FILT_EN   |   0x08
 *
 *  @note The user must select one among the following macros to
 *  select down-sampling ratio for gyro
 *      fifo_down                    |   Value
 *    -------------------------------|---------------------------
 *      BMG250_GYRO_FIFO_DOWN_ZERO   |   0x00
 *      BMG250_GYRO_FIFO_DOWN_ONE    |   0x01
 *      BMG250_GYRO_FIFO_DOWN_TWO    |   0x02
 *      BMG250_GYRO_FIFO_DOWN_THREE  |   0x03
 *      BMG250_GYRO_FIFO_DOWN_FOUR   |   0x04
 *      BMG250_GYRO_FIFO_DOWN_FIVE   |   0x05
 *      BMG250_GYRO_FIFO_DOWN_SIX    |   0x06
 *      BMG250_GYRO_FIFO_DOWN_SEVEN  |   0x07
 *
 *  @note : By ORing the above mentioned macros, the user can select
 *  the required FIFO down config settings
 *
 *  @param[in] dev : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_fifo_down(uint8_t fifo_down, const struct bmg250_dev *dev);

/*!
 *  @brief This API is used to set the FIFO watermark level in the sensor
 *
 *  @param[in] wm_frame_count : Variable used to set the FIFO watermark level
 *  @param[out] fifo_length   : Number of bytes in FIFO which is to be given
 *                              as input to dev->fifo->length
 *                              while calling "bmg250_get_fifo_data"
 *  @param[in] dev            : Structure instance of bmg250_dev.
 *
 *  @note The FIFO watermark is issued when the FIFO fill level is
 *  equal or above the watermark level
 *
 *  @note User can set "wm_frame_count" as the data frames he needs.
 *  wm_frame_count = 3 gives a watermark interrupt after
 *  3 valid data frames are obtained (1 frame = 6 bytes)
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_fifo_wm(uint8_t wm_frame_count, uint16_t *fifo_length, const struct bmg250_dev *dev);

/*!
 *  @brief This API reads the FIFO data from the sensor
 *
 *  @note User has to allocate the FIFO buffer along with
 *  corresponding fifo length from his side before calling this API
 *  as mentioned in the readme.md
 *
 *  @note User must specify the number of bytes to read from the FIFO in
 *  dev->fifo->length , It will be updated by the number of bytes actually
 *  read from FIFO after calling this API
 *
 *  @param[in] dev     : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_get_fifo_data(const struct bmg250_dev *dev);

/*!
 *  @brief This API parses and extracts the gyro frames from
 *  FIFO data read by the "bmg250_get_fifo_data" API and stores it in
 *  the "gyro_data" structure instance.
 *
 *  @note The bmg250_extract_gyro API should be called only after
 *  reading the FIFO data by calling the bmg250_get_fifo_data() API.
 *
 *  @param[out] gyro_data       : Structure instance of bmg250_sensor_data
 *                                where the gyro data in FIFO is stored.
 *  @param[in,out] data_length  : Number of gyro frames (x,y,z axes data)
 *                                user needs
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 *  @note data_length is updated with the number of valid gyro
 *  frames extracted from fifo (1 gyro frame = 6 bytes) at the end of
 *  execution of this API.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_extract_gyro(struct bmg250_sensor_data *gyro_data, uint8_t *data_length, const struct bmg250_dev *dev);

/*!
 *  @brief This API triggers the self test of the sensor and gives the self
 *  test result to the user as return value
 *
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 *  @return Result of API execution status and self test result.
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 *
 *        Return value         | Result of self test
 *   --------------------------|---------------------------------
 *    BMG250_OK                | Self test success
 *    BMG250_W_SELF_TEST_FAIL  | Self test failure
 */
int8_t bmg250_perform_self_test(const struct bmg250_dev *dev);

/*!
 *  @brief This API sets the watch-dog timer(WDT) enable/disable
 *  and the timer period of the WDT
 *
 *  @param[in] wdt_en       : Enable/disable watch-dog timer
 *  @param[in] wdt_sel      : Timer period for watch-dog timer
 *  @param[in] dev          : Structure instance of bmg250_dev.
 *
 *  @note possible settings of "wdt_en"  :
 *       - BMG250_DISABLE
 *       - BMG250_ENABLE
 *        possible settings of "wdt_sel" :
 *       - BMG250_I2C_WDT_1_MS
 *       - BMG250_I2C_WDT_50_MS
 *
 *  @note WDT can be enabled only when the interface is I2C
 *         and the timer period of 1ms or 50ms will be valid
 *         only when WDT is enabled
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_i2c_wdt_settings(uint8_t wdt_en, uint8_t wdt_sel, const struct bmg250_dev *dev);

/*!
 *  @brief This API triggers the fast offset compensation (FOC) in the sensor
 *
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 *  @note : Prerequisite of calling this API is that the power mode
 *  of the sensor should be set to normal mode by using the
 *  "bmg250_set_power_mode" API
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_foc(const struct bmg250_dev *dev);

/*!
 *  @brief This API is used to set the offsets for x,y,z axes of the sensor
 *
 *  @param[in] offset           : Structure instance of user defined offset.
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 *  @note : This API must be used to set the user defined offsets
 *  for all 3 axes A prerequisite for calling this API is that user must
 *  specify the offset in "struct bmg250_offset"
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_manual_offset(const struct bmg250_offset *offset, const struct bmg250_dev *dev);

/*!
 *  @brief This API is used to set interface mode of the sensor
 *
 *  @param[in] if_mode          : primary and secondary interface mode select
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 *   Value of if_mode           |  Interface selected
 *  ----------------------------|----------------------------------------
 *     BMG250_IF_AUTO_CONFIG    | Primary - autoconfig / Secondary - OFF
 *     BMG250_IF_I2C_OIS        | Primary - I2C / Secondary interface OIS
 *
 *   Value of spi_en            |  Interface selected
 *  ----------------------------|----------------------------------------
 *     BMG250_ENABLE            | SPI interface enabled
 *     BMG250_DISABLE           | SPI interface not enabled
 *
 *  @note SPI interface can only be enabled when the
 *        if_mode = BMG250_IF_AUTO_CONFIG and once enabled
 *        can be disabled by soft-reset or power-on-reset
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_set_if_mode(uint8_t if_mode, uint8_t spi_en, const struct bmg250_dev *dev);

/*!
 *  @brief This API is used to get the temperature data from the sensor
 *
 *  @param[in] temperature      : Temperature data from sensor
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 *  @note Value of temperature is in units of 1/1000(degree celcius)
 *  ie If value of temperature = 23996 ,
 *  Then temperature is 23.996 degree celcius
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bmg250_get_temperature(int32_t *temperature, const struct bmg250_dev *dev);

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif /* BMG250_H_ */
/** @}*/
