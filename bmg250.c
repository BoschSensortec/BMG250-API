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
 * @file    bmg250.c
 * @date    Oct 22 2018
 * @version 1.1.0
 * @brief   Sensor driver for BMG250 sensor
 *
 */

/*!
 * @defgroup bmg250
 * @brief
 * @{*/

#include "bmg250.h"

/*********************************************************************/
/* Static function declarations */

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev    : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t null_ptr_check(const struct bmg250_dev *dev);

/*!
 * @brief This internal API is used to set the appropriate
 * delay for power mode changes
 *
 * @param[in] gyro_pmu_status : previous power mode of the sensor
 * @param[in] dev             : Structure instance of bmg250_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static void power_mode_set_delay(const uint8_t *gyro_pmu_status, const struct bmg250_dev *dev);

/*!
 * @brief This internal API checks the invalid settings for ODR & BW
 *
 * @param[in] dev  :  Structure instance of bmg250_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error.
 */
static int8_t check_invalid_settg(const struct bmg250_dev *dev);

/*!
 * @brief This internal API sets the data ready interrupt
 * This interrupt occurs when new gyro data is available.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_data_ready_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API sets the FIFO full interrupt
 * This interrupt occurs when FIFO is full.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_fifo_full_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API sets the FIFO watermark interrupt
 * This interrupt occurs when the set FIFO watermark level is reached.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_fifo_watermark_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API enables the FIFO watermark interrupt.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t enable_fifo_wm_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API enables the FIFO full interrupt.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t enable_fifo_full_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API enables the data ready interrupt.
 *
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success  / -ve value -> Error
 */
static int8_t enable_data_ready(const struct bmg250_dev *dev);

/*!
 * @brief This internal API sets the interrupt pin configurations
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_intr_pin_config(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API sets the interrupt pin configurations
 * like interrupt channel output enable/disable, Push-pull/open-drain,
 * Active low/high, edge/level triggered
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t config_int_out_ctrl(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API enables/disables interrupt pin
 * to act as input to the sensor
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t config_int_input(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API maps the data ready interrupt to INT pin as per selection.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t map_data_ready_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API maps the FIFO full interrupt to INT pin as per selection.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t map_int_pin_to_fifo_full(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 * @brief This internal API maps the FIFO watermark interrupt to INT pin as per selection.
 *
 * @param[in] int_config  : Structure instance of bmg250_int_settg.
 * @param[in] dev         : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t map_int_pin_to_fifo_wtm(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to reset the FIFO related configurations
 *  in the bmg250_fifo_frame structure.
 *
 * @param[in] dev             : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static void reset_fifo_data_structure(const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to read fifo_byte_counter value (i.e)
 *  current fill-level in FIFO buffer.
 *
 * @param[out] bytes_to_read  : Number of bytes available in FIFO at the
 *                              instant obtained from FIFO counter.
 * @param[in] dev             : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, const struct bmg250_dev *dev);

/*!
 *  @brief This internal API computes the number of bytes of gyro FIFO data
 *  which is to be parsed in header-less mode
 *
 *  @param[out] data_index       : The start index for parsing data
 *  @param[out] data_read_length : No of bytes to be parsed from FIFO buffer
 *  @param[in] gyro_frame_count  : Number of Gyro data frames to be read
 *  @param[in] dev               : Structure instance of bmg250_dev.
 */
static void get_gyro_len_to_parse(uint16_t *data_index,
	uint16_t *data_read_length,
	const uint8_t *gyro_frame_count,
	const struct bmg250_dev *dev);

/*!
 *  @brief This internal API checks the presence of non-valid frames in the read fifo data.
 *
 *  @param[in,out] data_index	 : The index of the current data to
 *                                 be parsed from fifo data
 *  @param[in] dev               : Structure instance of bmg250_dev.
 */
static void check_frame_validity(uint16_t *data_index, const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to parse the gyroscope's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 *  @param[in,out] gyro		: structure instance of sensor data
 *  @param[in,out] idx		: Index value of number of bytes parsed
 *  @param[in,out] gyro_idx	: Index value of gyro data
 *                                (x,y,z axes) frames parsed
 *  @param[in] frame_info       : It consists of either fifo_data_enable
 *                                parameter in header-less mode or
 *                                frame header data in header mode
 *  @param[in] dev		: structure instance of bmg250_dev.
 */
static void unpack_gyro_frame(struct bmg250_sensor_data *gyro,
	uint16_t *idx,
	uint8_t *gyro_idx,
	uint8_t frame_info,
	const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to parse the gyro data from the
 *  FIFO data and store it in the instance of the structure bmg250_sensor_data.
 *
 *  @param[in,out] gyro_data         : structure instance of sensor data
 *  @param[in,out] data_start_index  : Index value of number of bytes parsed
 *  @param[in] dev		     : structure instance of bmg250_dev.
 */
static void unpack_gyro_data(struct bmg250_sensor_data *gyro_data,
	uint16_t data_start_index,
	const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to parse the gyro data from the
 *  FIFO data in header mode.
 *
 *  @param[in,out] gyro_data	 : Structure instance of sensor data
 *  @param[in,out] gyro_length   : Number of gyro frames
 *  @param[in] dev               : Structure instance of bmg250_dev.
 */
static void extract_gyro_header_mode(struct bmg250_sensor_data *gyro_data,
	uint8_t *gyro_length,
	const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev->fifo->sensor_time.
 *
 *  @param[in,out] data_index : Index of the FIFO data which
 *                               has the sensor time.
 *  @param[in] dev            : Structure instance of bmg250_dev.
 *
 */
static void unpack_sensortime_frame(uint16_t *data_index, const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev->fifo->skipped_frame_count.
 *
 *  @param[in,out] data_index   : Index of the FIFO data which
 *                                has the skipped frame count.
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 */
static void unpack_skipped_frame(uint16_t *data_index, const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                    is to be moved ahead of the
 *                                    current_frame_length
 *  @param[in] current_frame_length : Number of bytes in a particular frame
 *  @param[in] dev                  : Structure instance of bmg250_dev.
 *
 */
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to enable the sensor to write the
 *  offset for data compensation
 *
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t enable_gyro_offset(const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to set the offset values for all the 3 axes of the sensor.
 *
 *  @param[in] offset           : Structure instance of offset as specified by user.
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_gyro_offset(const struct bmg250_offset *offset, const struct bmg250_dev *dev);

/*!
 *  @brief This internal API is used to enable the SPI interface in the sensor
 *
 *  @param[in] dev              : Structure instance of bmg250_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t set_spi_enable(const struct bmg250_dev *dev);

/*********************************************************************/
/* User function definitions  */

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of bmg250 sensor.
 */
int8_t bmg250_init(struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data;
	uint8_t chip_id;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if ((rslt == BMG250_OK) && (dev->intf == BMG250_SPI_INTF)) {
		/* Dummy read of 0x7F register to enable SPI Interface
		   if SPI is used */
		rslt = bmg250_get_regs(BMG250_SPI_COMM_TEST_ADDR, &data, 1, dev);
	}
	if (rslt == BMG250_OK) {
		/* Read chip_id */
		rslt = bmg250_get_regs(BMG250_CHIP_ID_ADDR, &chip_id, 1, dev);
		if ((rslt == BMG250_OK) && (chip_id == BMG250_CHIP_ID)) {
			/* Updating chip_id in device structure */
			dev->chip_id = chip_id;
		} else {
			rslt = BMG250_E_DEV_NOT_FOUND;
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address
 * of sensor.
 */
int8_t bmg250_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if ((rslt == BMG250_OK) && (data != NULL)) {
		if (dev->intf == BMG250_SPI_INTF) {
			/* Configuring reg_addr for SPI Interface */
			reg_addr = (reg_addr | BMG250_SPI_RD_MASK);
		}

		/* Perform the read operation */
		rslt = dev->read(dev->dev_id, reg_addr, data, len);

		/* Delay for proper data read */
		dev->delay_ms(1);
		if (rslt != BMG250_OK) {
			/* Failure case */
			rslt = BMG250_E_COM_FAIL;
		}
	} else {
		rslt = BMG250_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address of sensor.
 */
int8_t bmg250_set_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint16_t count = 0;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);

	/* Check for null pointer in the device structure*/
	if ((rslt == BMG250_OK) && (data != NULL)) {
		if (dev->intf == BMG250_SPI_INTF) {
			/* Configuring reg_addr for SPI Interface */
			reg_addr = (reg_addr & BMG250_SPI_WR_MASK);
		}

		/* Burst write is allowed only in normal mode */
		if (dev->power_mode == BMG250_GYRO_NORMAL_MODE) {
			rslt = dev->write(dev->dev_id, reg_addr, data, len);
			dev->delay_ms(1);
		} else {
			/* Burst write is not allowed in
			   suspend & fast-startup mode */
			for (; count < len; count++) {
				rslt = dev->write(dev->dev_id, reg_addr, &data[count], 1);
				reg_addr++;

				/* Interface Idle time delay
				   for proper write operation */
				dev->delay_ms(1);
			}
		}
		if (rslt != BMG250_OK) {
			/* Failure case */
			rslt = BMG250_E_COM_FAIL;
		}
	} else {
		rslt = BMG250_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API resets and restarts the device.
 * All register values are overwritten with default parameters.
 */
int8_t bmg250_soft_reset(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data = BMG250_SOFT_RESET_CMD;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Reset the device */
		rslt = bmg250_set_regs(BMG250_COMMAND_REG_ADDR, &data, 1, dev);
		dev->delay_ms(BMG250_SOFT_RESET_DELAY_MS);
		if ((rslt == BMG250_OK) && (dev->intf == BMG250_SPI_INTF)) {
			/* Dummy read of 0x7F register to enable SPI Interface
			   if SPI is used */
			rslt = bmg250_get_regs(BMG250_SPI_COMM_TEST_ADDR, &data, 1, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t bmg250_set_power_mode(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t gyro_pmu_status = 0;
	uint8_t power_mode_set;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		if ((dev->power_mode == BMG250_GYRO_SUSPEND_MODE) || (dev->power_mode == BMG250_GYRO_NORMAL_MODE) ||
			(dev->power_mode == BMG250_GYRO_FASTSTARTUP_MODE)) {
			/* Read the existing PMU status & store it */
			rslt = bmg250_get_regs(BMG250_PMU_STATUS_ADDR, &gyro_pmu_status, 1, dev);
			if (rslt == BMG250_OK) {
				/* Set the user specified gyro power mode in the cmd register */
				power_mode_set = dev->power_mode;
				rslt = bmg250_set_regs(BMG250_COMMAND_REG_ADDR, &power_mode_set, 1, dev);
				if (rslt == BMG250_OK) {
					/* Previous power mode is checked
					   to provide appropriate delay */
					gyro_pmu_status = BMG250_GET_BITS(gyro_pmu_status, BMG250_GYRO_PMU_STATUS);
					power_mode_set_delay(&gyro_pmu_status, dev);
				}
			}
		} else {
			rslt = BMG250_E_OUT_OF_RANGE;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the sensor configurations like ODR, Range, BW.
 */
int8_t bmg250_set_sensor_settings(const struct bmg250_cfg *gyro_cfg, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data[2] = { 0 };

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Read gyro output data rate and bandwidth */
		rslt = bmg250_get_regs(BMG250_GYRO_CONFIG_ADDR, data, 2, dev);
		if (rslt == BMG250_OK) {
			/* Setting output data rate */
			data[0] = BMG250_SET_BITS_POS_0(data[0], BMG250_GYRO_ODR, gyro_cfg->odr);

			/* Setting bandwidth */
			data[0] = BMG250_SET_BITS(data[0], BMG250_GYRO_BW, gyro_cfg->bw);

			/* Setting range */
			data[1] = BMG250_SET_BITS_POS_0(data[1], BMG250_GYRO_RANGE, gyro_cfg->range);

			/* Set gyro output data rate and bandwidth */
			rslt = bmg250_set_regs(BMG250_GYRO_CONFIG_ADDR, data, 2, dev);
			if (rslt == BMG250_OK) {
				/* Check for invalid setting */
				rslt = check_invalid_settg(dev);
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API gets the sensor configurations like
 * ODR, Range, BW from the sensor
 */
int8_t bmg250_get_sensor_settings(struct bmg250_cfg *gyro_cfg, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data[2] = { 0 };

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* read gyro output data rate and bandwidth */
		rslt = bmg250_get_regs(BMG250_GYRO_CONFIG_ADDR, data, 2, dev);

		/* Extract the BW, ODR, Range */
		gyro_cfg->bw = BMG250_GET_BITS(data[0], BMG250_GYRO_BW);
		gyro_cfg->odr = BMG250_GET_BITS_POS_0(data[0], BMG250_GYRO_ODR);
		gyro_cfg->range = BMG250_GET_BITS_POS_0(data[1], BMG250_GYRO_RANGE);
	}

	return rslt;
}

/*!
 * @brief This API reads gyro data along with sensor time if time is requested
 * by user. Kindly refer the user guide(README.md) for more info.
 */
int8_t bmg250_get_sensor_data(uint8_t data_sel, struct bmg250_sensor_data *gyro, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t idx = 0;
	uint8_t data_array[9] = { 0 };
	uint32_t time_0 = 0;
	uint32_t time_1 = 0;
	uint32_t time_2 = 0;
	uint8_t lsb;
	uint8_t msb;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if ((rslt == BMG250_OK) && (gyro != NULL)) {
		if (data_sel == BMG250_DATA_SEL) {
			/* read gyro sensor data alone */
			rslt = bmg250_get_regs(BMG250_DATA_ADDR, data_array, 6, dev);
			if (rslt == BMG250_OK) {
				lsb = data_array[idx++];
				msb = data_array[idx++];

				/* gyro X axis data */
				gyro->x = (int16_t)(((uint16_t)msb << 8) | lsb);
				lsb = data_array[idx++];
				msb = data_array[idx++];

				/* gyro Y axis data */
				gyro->y = (int16_t)(((uint16_t)msb << 8) | lsb);
				lsb = data_array[idx++];
				msb = data_array[idx++];

				/* gyro Z axis data */
				gyro->z = (int16_t)(((uint16_t)msb << 8) | lsb);

				/* update sensor-time data as 0 */
				gyro->sensortime = 0;
			}
		} else if (data_sel == BMG250_DATA_TIME_SEL) {
			/* read gyro sensor data along with sensor-time */
			rslt = bmg250_get_regs(BMG250_DATA_ADDR, data_array, 9, dev);
			if (rslt == BMG250_OK) {
				lsb = data_array[idx++];
				msb = data_array[idx++];

				/* gyro X axis data */
				gyro->x = (int16_t)(((uint16_t)msb << 8) | lsb);
				lsb = data_array[idx++];
				msb = data_array[idx++];

				/* gyro Y axis data */
				gyro->y = (int16_t)(((uint16_t)msb << 8) | lsb);
				lsb = data_array[idx++];
				msb = data_array[idx++];

				/* gyro Z axis data */
				gyro->z = (int16_t)(((uint16_t)msb << 8) | lsb);

				/* Sensor-time data*/
				time_0 = data_array[idx++];
				time_1 = ((uint32_t)data_array[idx++] << 8);
				time_2 = ((uint32_t)data_array[idx++] << 16);
				gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
			}
		} else {
			rslt = BMG250_E_INVALID_INPUT;
		}
	} else {
		rslt = BMG250_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API configures the necessary interrupt based on
 *  the user settings in the bmg250_int_settg structure instance.
 */
int8_t bmg250_set_int_config(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if ((rslt == BMG250_OK) && (int_config != NULL)) {
		switch (int_config->int_type) {
		case BMG250_DATA_RDY_INT:

			/* Data ready interrupt */
			rslt = set_data_ready_int(int_config, dev);
			break;
		case BMG250_FIFO_FULL_INT:

			/* FIFO full interrupt */
			rslt = set_fifo_full_int(int_config, dev);
			break;
		case BMG250_FIFO_WATERMARK_INT:

			/* FIFO water-mark interrupt */
			rslt = set_fifo_watermark_int(int_config, dev);
			break;
		default:
			break;
		}
	} else {
		rslt = BMG250_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API gives us the status of the interrupts
 * whether they are triggered or not
 */
int8_t bmg250_get_int_status(uint8_t *int_status, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* read the error reg */
		rslt = bmg250_get_regs(BMG250_INT_STATUS_ADDR, int_status, 1, dev);
	} else {
		rslt = BMG250_E_NULL_PTR;
	}

	return rslt;
}

/*! @brief This API sets the FIFO configuration in the sensor.
 */
int8_t bmg250_set_fifo_config(uint8_t config, uint8_t enable, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Read the FIFO_CONFIG_1 register */
		rslt = bmg250_get_regs(BMG250_FIFO_CONFIG1_ADDR, &reg_data, 1, dev);
		if (rslt == BMG250_OK) {
			/* Check for configuration selected */
			if (config > 0) {
				if (enable == BMG250_ENABLE) {
					/* Enable the FIFO setting*/
					reg_data = reg_data | config;
				} else {
					reg_data = reg_data & (~config);
				}
			}

			/* Set the FIFO config in the sensor */
			rslt = bmg250_set_regs(BMG250_FIFO_CONFIG1_ADDR, &reg_data, 1, dev);
			if (rslt == BMG250_OK) {
				/* Read the FIFO_CONFIG_1 register */
				rslt = bmg250_get_regs(BMG250_FIFO_CONFIG1_ADDR, &reg_data, 1, dev);
				if (rslt == BMG250_OK) {
					/* Extract fifo header enabled status */
					dev->fifo->fifo_header_enable = BMG250_GET_BITS(reg_data, BMG250_FIFO_HEADER_EN);

					/* Extract fifo sensor time enabled status */
					dev->fifo->fifo_time_enable = BMG250_GET_BITS(reg_data, BMG250_FIFO_TIME_EN);

					/* Extract data enabled status */
					dev->fifo->fifo_data_enable = BMG250_GET_BITS(reg_data, BMG250_FIFO_DATA_EN);
				}
			}
		}
	} else {
		rslt = BMG250_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API writes fifo_flush command to command register.
 *  This action clears all data in the sensor's FIFO without changing
 *  FIFO configuration settings
 */
int8_t bmg250_set_fifo_flush(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data = BMG250_FIFO_FLUSH_CMD;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Clears all data in FIFO of sensor */
		rslt = bmg250_set_regs(BMG250_COMMAND_REG_ADDR, &data, 1, dev);
	}

	return rslt;
}

/*!
 *  @brief This API is used to configure the down sampling ratios
 *  for sensor data in FIFO.
 *  It also configures either filtered/pre-filtered data to
 *  be available in FIFO
 */
int8_t bmg250_set_fifo_down(uint8_t fifo_down, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data = 0;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Read the FIFO_DOWN register */
		rslt = bmg250_get_regs(BMG250_FIFO_DOWN_ADDR, &reg_data, 1, dev);
		if (rslt == BMG250_OK) {
			reg_data = BMG250_SET_BITS_POS_0(reg_data, BMG250_FIFO_DOWN_SAMPLING, fifo_down);
			rslt = bmg250_set_regs(BMG250_FIFO_DOWN_ADDR, &reg_data, 1, dev);
		}
	}

	return rslt;
}

/*!
 *  @brief This API is used to set the FIFO watermark level in the sensor
 */
int8_t bmg250_set_fifo_wm(uint8_t wm_frame_count, uint16_t *fifo_length, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint16_t fifo_wm;
	uint8_t wm_reg_val;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		if (dev->fifo->fifo_header_enable == BMG250_ENABLE) {
			/* FIFO watermark is found in terms of bytes */
			fifo_wm = (wm_frame_count * BMG250_FIFO_G_HEADER_LENGTH);
		} else {
			fifo_wm = (wm_frame_count * BMG250_FIFO_G_LENGTH);
		}

		/* Handling out of range request in setting water-mark */
		if (fifo_wm > 996) {
			/* Setting Watrmark to 996 bytes */
			fifo_wm = 996;
		}

		/* FIFO length must be returned to the user */
		*fifo_length = fifo_wm;

		/* The unit of FIFO watermark is in terms of 4 bytes */
		wm_reg_val = (uint8_t)(fifo_wm / 4);
		if ((fifo_wm % 4 != 0) && ((fifo_wm + 4) < 996)) {
			/* Additional 4 bytes are added */
			wm_reg_val = wm_reg_val + 1;
		}

		/* Set the FIFO watermark */
		rslt = bmg250_set_regs(BMG250_FIFO_CONFIG0_ADDR, &wm_reg_val, 1, dev);
	}

	return rslt;
}

/*!
 *  @brief This API reads the FIFO data from the sensor
 */
int8_t bmg250_get_fifo_data(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t addr = BMG250_FIFO_DATA_ADDR;
	uint16_t bytes_to_read = 0;
	uint16_t user_fifo_len = 0;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		reset_fifo_data_structure(dev);
		rslt = get_fifo_byte_counter(&bytes_to_read, dev);
		if (rslt == BMG250_OK) {
			user_fifo_len = dev->fifo->length;
			if (dev->fifo->length > bytes_to_read) {
				/* Handling the case where user requests
				   more data than available in FIFO */
				dev->fifo->length = bytes_to_read;
			}
			if ((dev->fifo->fifo_time_enable == BMG250_ENABLE) && (bytes_to_read + 4 <= user_fifo_len)) {
				/* Handling case of sensor time availability */
				dev->fifo->length = dev->fifo->length + 4;
			}
			if (dev->intf == BMG250_SPI_INTF) {
				/* SPI mask for reading FIFO */
				addr = addr | BMG250_SPI_RD_MASK;
			}

			/* Read only the filled bytes in the FIFO Buffer */
			rslt = dev->read(dev->dev_id, addr, dev->fifo->data, dev->fifo->length);
		}
	}

	return rslt;
}

/*!
 *  @brief This API parses and extracts the gyro frames from
 *  FIFO data read by the "bmg250_get_fifo_data" API and stores it in
 *  the "gyro_data" structure instance.
 */
int8_t bmg250_extract_gyro(struct bmg250_sensor_data *gyro_data, uint8_t *data_length, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint16_t data_index = 0;
	uint16_t data_read_length = 0;
	uint8_t gyro_index = 0;
	uint8_t fifo_data_enable = 0;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Parsing the FIFO data in header-less mode */
		if (dev->fifo->fifo_header_enable == 0) {
			/* Number of bytes to be parsed from FIFO */
			get_gyro_len_to_parse(&data_index, &data_read_length, data_length, dev);
			for (; data_index < data_read_length;) {
				/*Check for the availability of next two bytes of FIFO data */
				check_frame_validity(&data_index, dev);
				fifo_data_enable = dev->fifo->fifo_data_enable;
				unpack_gyro_frame(gyro_data, &data_index, &gyro_index, fifo_data_enable, dev);
			}

			/* update number of gyro data read */
			*data_length = gyro_index;

			/* update the gyro byte index */
			dev->fifo->gyro_byte_start_idx = data_index;
		} else {
			/* Parsing the FIFO data in header mode */
			extract_gyro_header_mode(gyro_data, &gyro_index, dev);
			*data_length = gyro_index;
		}
	}

	return rslt;
}

/*!
 *  @brief This API triggers the self test of the sensor and gives the self
 *  test result to the user as return value
 */
int8_t bmg250_perform_self_test(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t self_test_trigger;
	uint8_t self_test_rslt;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Trigger the self test */
		self_test_trigger = BMG250_SELF_TEST_TRIGGER;
		rslt = bmg250_set_regs(BMG250_SELF_TEST_ADDR, &self_test_trigger, 1, dev);
		if (rslt == BMG250_OK) {
			/* Read the self test status */
			dev->delay_ms(20);
			rslt = bmg250_get_regs(BMG250_STATUS_ADDR, &self_test_rslt, 1, dev);
			if (rslt == BMG250_OK) {
				/* Extract the self test status bit */
				self_test_rslt = BMG250_GET_BITS(self_test_rslt, BMG250_SELF_TEST_RSLT);
				if (self_test_rslt == BMG250_ENABLE) {
					/* Self test success */
					rslt = BMG250_OK;
				} else {
					rslt = BMG250_W_SELF_TEST_FAIL;
				}
			}
		}
	}

	return rslt;
}

/*!
 *  @brief This API sets the watch-dog timer enable/disable and timer period
 */
int8_t bmg250_set_i2c_wdt_settings(uint8_t wdt_en, uint8_t wdt_sel, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Read the watch-dog timer settings register */
		rslt = bmg250_get_regs(BMG250_NV_CONF_ADDR, &reg_data, 1, dev);
		if (rslt == BMG250_OK) {
			/* Set the required WDT settings */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_WDT_EN, wdt_en);
			if (wdt_en == BMG250_ENABLE) {
				/* Set required WDT period if WDT is enabled*/
				reg_data = BMG250_SET_BITS(reg_data, BMG250_WDT_SEL, wdt_sel);
			}

			/* Set the WDT config in the sensor */
			rslt = bmg250_set_regs(BMG250_NV_CONF_ADDR, &reg_data, 1, dev);
		}
	}

	return rslt;
}

/*!
 *  @brief This API triggers the fast offset compensation (FOC) in the sensor
 */
int8_t bmg250_set_foc(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;
	uint8_t data[6];

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Enable the gyro offset */
		rslt = enable_gyro_offset(dev);
		if (rslt == BMG250_OK) {
			reg_data = BMG250_FOC_ENABLE_CMD;
			rslt = bmg250_set_regs(BMG250_COMMAND_REG_ADDR, &reg_data, 1, dev);
			if (rslt == BMG250_OK) {
				dev->delay_ms(BMG250_FOC_DELAY_MS);

				/* Read the FOC status */
				rslt = bmg250_get_regs(BMG250_STATUS_ADDR, &reg_data, 1, dev);
				reg_data = BMG250_GET_BITS(reg_data, BMG250_FOC_READY);
				if ((rslt == BMG250_OK) && (reg_data == BMG250_ENABLE)) {
					/* Step to remove stall data-ready bit */
					rslt = bmg250_get_regs(BMG250_DATA_ADDR, &data[0], 6, dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 *  @brief This API is used to set the offsets for x,y,z axes of the sensor
 */
int8_t bmg250_set_manual_offset(const struct bmg250_offset *offset, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Enable the gyro offset */
		rslt = enable_gyro_offset(dev);
		if (rslt == BMG250_OK) {
			/* Set the user specified offset in the sensor */
			rslt = set_gyro_offset(offset, dev);
		}
	}

	return rslt;
}

/*!
 *  @brief This API is used to set interface mode of the sensor
 */
int8_t bmg250_set_if_mode(uint8_t if_mode, uint8_t spi_en, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Read the IF_CONF register */
		rslt = bmg250_get_regs(BMG250_IF_CONF_ADDR, &reg_data, 1, dev);
		if (rslt == BMG250_OK) {
			reg_data = BMG250_SET_BITS(reg_data, BMG250_IF_CONFIG, if_mode);

			/* Set the IF_CONF in the sensor */
			rslt = bmg250_set_regs(BMG250_IF_CONF_ADDR, &reg_data, 1, dev);
			if (rslt == BMG250_OK) {
				if ((if_mode == BMG250_IF_AUTO_CONFIG) && (spi_en == BMG250_ENABLE)) {
					/* Enable the SPI interface */
					rslt = set_spi_enable(dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 *  @brief This API is used to get the temperature data from the sensor
 */
int8_t bmg250_get_temperature(int32_t *temperature, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data[2];
	uint16_t temp_data;

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Read the temperature data registers */
		rslt = bmg250_get_regs(BMG250_TEMPERATURE_ADDR, data, 2, dev);
		if (rslt == BMG250_OK) {
			temp_data = ((uint16_t)data[1] << 8) | data[0];
			if (temp_data == 0x8000) {
				rslt = BMG250_E_INVALID_TEMPERATURE;
			} else {
				*temperature = (((int16_t) temp_data * INT32_C(1000)) / INT32_C(512)) + INT32_C(23000);
			}
		}
	}

	return rslt;
}

/*********************************************************************/
/* Static function definitions */

/*!
 * @brief This internal API is used to validate the device structure pointer
 * for null conditions.
 */
static int8_t null_ptr_check(const struct bmg250_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		rslt = BMG250_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = BMG250_OK;
	}

	return rslt;
}

/*!
 * @brief This internal API is used to set the appropriate
 * delay for power mode changes
 */
static void power_mode_set_delay(const uint8_t *gyro_pmu_status, const struct bmg250_dev *dev)
{
	/* startup time delay */
	if (*gyro_pmu_status == BMG250_PMU_STATUS_SUSPEND) {
		/* Delay of 81 ms */
		dev->delay_ms(BMG250_GYRO_DELAY_MS);
	} else if (*gyro_pmu_status == BMG250_PMU_STATUS_FSU) {
		/* This delay is required for transition from
		   fast-startup mode to normal mode */
		dev->delay_ms(BMG250_GYRO_FSU_DELAY_MS);
	}
}

/*!
 * @brief This internal API checks the invalid settings for ODR & BW
 */
static int8_t check_invalid_settg(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data = 0;

	/* read the error reg */
	rslt = bmg250_get_regs(BMG250_ERROR_REG_ADDR, &data, 1, dev);

	/* ODR-BW invalid setting error is checked */
	data = data >> 1;
	data = data & BMG250_ERR_REG_MASK;
	if (data == 2) {
		/* Gyro config error */
		rslt = BMG250_E_ODR_BW_INVALID;
	}

	return rslt;
}

/*!
 * @brief This internal API sets the data ready interrupt
 * This interrupt occurs when new gyro data is available.
 */
static int8_t set_data_ready_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* Enable data ready interrupt engine */
	rslt = enable_data_ready(dev);
	if (rslt == BMG250_OK) {
		/* Sets the desired interrupt pin configurations */
		rslt = set_intr_pin_config(int_config, dev);
		if (rslt == BMG250_OK) {
			/* Map the interrupt to INT1/INT2 pin of sensor */
			rslt = map_data_ready_int(int_config, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API sets the FIFO full interrupt
 * This interrupt occurs when FIFO is full.
 */
static int8_t set_fifo_full_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* Enable the fifo full interrupt */
	rslt = enable_fifo_full_int(int_config, dev);
	if (rslt == BMG250_OK) {
		/* Sets the desired interrupt pin configurations */
		rslt = set_intr_pin_config(int_config, dev);
		if (rslt == BMG250_OK) {
			/* Map the interrupt to INT1/INT2 pin of sensor */
			rslt = map_int_pin_to_fifo_full(int_config, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API sets the FIFO watermark interrupt
 * This interrupt occurs when the set FIFO watermark level is reached.
 */
static int8_t set_fifo_watermark_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* Enable FIFO watermark interrupt */
	rslt = enable_fifo_wm_int(int_config, dev);
	if (rslt == BMG250_OK) {
		/* Sets the desired interrupt pin configurations */
		rslt = set_intr_pin_config(int_config, dev);
		if (rslt == BMG250_OK) {
			/* Map the interrupt to INT1/INT2 pin of sensor */
			rslt = map_int_pin_to_fifo_wtm(int_config, dev);
		}
	}

	return rslt;
}

/*!
 * @brief This API enables the FIFO watermark interrupt.
 */
static int8_t enable_fifo_wm_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Enable FIFO watermark interrupt in INT_EN register */
	rslt = bmg250_get_regs(BMG250_INT_EN_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		reg_data = BMG250_SET_BITS(reg_data, BMG250_FIFO_WM_EN, int_config->fifo_wtm_int_en);

		/* Writing data to INT_ENABLE Address */
		rslt = bmg250_set_regs(BMG250_INT_EN_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This API enables the FIFO full interrupt.
 */
static int8_t enable_fifo_full_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Enable FIFO full interrupt in INT_EN register */
	rslt = bmg250_get_regs(BMG250_INT_EN_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		reg_data = BMG250_SET_BITS(reg_data, BMG250_FIFO_FULL_EN, int_config->fifo_full_int_en);

		/* Writing data to INT_ENABLE Address */
		rslt = bmg250_set_regs(BMG250_INT_EN_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API enables the data ready interrupt.
 */
static int8_t enable_data_ready(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Enable data ready interrupt in INT_EN register */
	rslt = bmg250_get_regs(BMG250_INT_EN_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		reg_data = BMG250_SET_BITS(reg_data, BMG250_DATA_READY_EN, BMG250_ENABLE);

		/* Writing data to INT_ENABLE Address */
		rslt = bmg250_set_regs(BMG250_INT_EN_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the interrupt pin configurations
 */
static int8_t set_intr_pin_config(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;

	/* configure the behavioural settings of interrupt pin */
	rslt = config_int_out_ctrl(int_config, dev);
	if (rslt == BMG250_OK) {
		/*Configuring interrupt pin as input */
		rslt = config_int_input(int_config, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the interrupt pin configurations
 * like interrupt channel output enable/disable, Push-pull/open-drain,
 * Active low/high, edge/level triggered
 */
static int8_t config_int_out_ctrl(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Configuration of output interrupt signals on pins INT1 and INT2 are
	 * done in BMG250_IN_OUT_CNTRL_ADDR register*/
	rslt = bmg250_get_regs(BMG250_IN_OUT_CNTRL_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		/* Configuring channel 1 */
		if (int_config->int_channel == BMG250_INT_CHANNEL_1) {
			/* Output enable */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT1_EN, int_config->int_pin_settg.output_en);

			/* Output mode */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT1_OD, int_config->int_pin_settg.output_mode);

			/* Output type */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT1_LVL, int_config->int_pin_settg.output_type);

			/* edge control */
			reg_data = BMG250_SET_BITS_POS_0(reg_data, BMG250_INT1_EDGE, int_config->int_pin_settg.edge_ctrl);
		} else if (int_config->int_channel == BMG250_INT_CHANNEL_2) {
			/* Configuring channel 2 */
			/* Output enable */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_EN, int_config->int_pin_settg.output_en);

			/* Output mode */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_OD, int_config->int_pin_settg.output_mode);

			/* Output type */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_LVL, int_config->int_pin_settg.output_type);

			/* edge control */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_EDGE, int_config->int_pin_settg.edge_ctrl);
		}

		/* Set the configurations in the sensor */
		rslt = bmg250_set_regs(BMG250_IN_OUT_CNTRL_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API enables/disables interrupt pin
 * to act as input to the sensor
 */
static int8_t config_int_input(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	rslt = bmg250_get_regs(BMG250_INT_IN_CTRL_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		if (int_config->int_channel == BMG250_INT_CHANNEL_1) {
			/* Configuring channel 1 Input enable */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT1_INPUT_EN, int_config->int_pin_settg.input_en);
		} else {
			/* Configuring channel 2 Input enable */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_INPUT_EN, int_config->int_pin_settg.input_en);
		}

		/* Set the settings in the sensor */
		rslt = bmg250_set_regs(BMG250_INT_IN_CTRL_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This API maps the data ready interrupt to INT pin as per selection.
 */
static int8_t map_data_ready_int(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Configure Map register to map interrupt pin to data ready interrupt*/
	rslt = bmg250_get_regs(BMG250_INT_MAP_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		if (int_config->int_channel == BMG250_INT_CHANNEL_1) {
			/* Mapping to INT 1 pin */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT1_DATA_READY, BMG250_ENABLE);
		} else {
			/* Mapping to INT 2 pin */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_DATA_READY, BMG250_ENABLE);
		}

		/* Configure Map register to map interrupt pin to data ready interrupt*/
		rslt = bmg250_set_regs(BMG250_INT_MAP_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This API maps the FIFO full interrupt to INT pin as per selection.
 */
static int8_t map_int_pin_to_fifo_full(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Configure Map register to map interrupt pin to FIFO-full interrupt*/
	rslt = bmg250_get_regs(BMG250_INT_MAP_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		if (int_config->int_channel == BMG250_INT_CHANNEL_1) {
			/* Mapping to INT 1 pin */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT1_FIFO_FULL, BMG250_ENABLE);
		} else {
			/* Mapping to INT 2 pin */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_FIFO_FULL, BMG250_ENABLE);
		}

		/* Configure Map register to map interrupt pin to data ready interrupt*/
		rslt = bmg250_set_regs(BMG250_INT_MAP_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 * @brief This API maps the FIFO watermark interrupt to INT pin as per selection.
 */
static int8_t map_int_pin_to_fifo_wtm(const struct bmg250_int_settg *int_config, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Configure Map register to map interrupt pin to FIFO watermark interrupt*/
	rslt = bmg250_get_regs(BMG250_INT_MAP_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		if (int_config->int_channel == BMG250_INT_CHANNEL_1) {
			/* Mapping to INT 1 pin */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT1_FIFO_WM, BMG250_ENABLE);
		} else {
			/* Mapping to INT 2 pin */
			reg_data = BMG250_SET_BITS(reg_data, BMG250_INT2_FIFO_WM, BMG250_ENABLE);
		}

		/* Configure Map register to map interrupt pin to FIFO watermark interrupt*/
		rslt = bmg250_set_regs(BMG250_INT_MAP_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 *  @brief This API is used to reset the FIFO related configurations
 *  in the bmg250_fifo_frame structure.
 */
static void reset_fifo_data_structure(const struct bmg250_dev *dev)
{
	/* Prepare for next FIFO read by resetting FIFO's
	   internal data structures */
	dev->fifo->gyro_byte_start_idx = 0;
	dev->fifo->sensor_time = 0;
	dev->fifo->skipped_frame_count = 0;
}

/*!
 *  @brief This API is used to read fifo_byte_counter value (i.e)
 *  current fill-level in FIFO buffer.
 */
static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t data[2] = { 0 };

	rslt = bmg250_get_regs(BMG250_FIFO_LENGTH_ADDR, data, 2, dev);
	data[1] = data[1] & BMG250_FIFO_BYTE_COUNTER_MASK;

	/* Available data in FIFO is stored in bytes_to_read*/
	*bytes_to_read = (((uint16_t)data[1] << 8) | ((uint16_t)data[0]));

	return rslt;
}

/*!
 *  @brief This API computes the number of bytes of gyro FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_gyro_len_to_parse(uint16_t *data_index,
	uint16_t *data_read_length,
	const uint8_t *gyro_frame_count,
	const struct bmg250_dev *dev)
{
	/* Data start index */
	*data_index = dev->fifo->gyro_byte_start_idx;
	if (dev->fifo->fifo_data_enable == BMG250_ENABLE) {
		*data_read_length = (*gyro_frame_count) * BMG250_FIFO_G_LENGTH;
	} else {
		/* When gyro data is not enabled in FIFO,
		 * there will be no gyro data,
		 * so we update the data index as complete */
		*data_index = dev->fifo->length;
	}
	if (*data_read_length > dev->fifo->length) {
		/* Handling the case where more data is requested
		 * than that is available*/
		*data_read_length = dev->fifo->length;
	}
}

/*!
 *  @brief This API checks the presence of non-valid frames in the read fifo data.
 */
static void check_frame_validity(uint16_t *data_index, const struct bmg250_dev *dev)
{
	if ((*data_index + 2) < dev->fifo->length) {
		/* Check if FIFO is empty */
		if ((dev->fifo->data[*data_index] == FIFO_CONFIG_MSB_CHECK) &&
			(dev->fifo->data[*data_index + 1] == FIFO_CONFIG_LSB_CHECK)) {
			/* Update the data index as complete */
			*data_index = dev->fifo->length;
		}
	}
}

/*!
 *  @brief This API is used to parse the gyroscope's data from the
 *  FIFO data in both header mode and header-less mode.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_gyro_frame(struct bmg250_sensor_data *gyro,
	uint16_t *idx,
	uint8_t *gyro_idx,
	uint8_t frame_info,
	const struct bmg250_dev *dev)
{
	switch (frame_info) {
	case BMG250_FIFO_HEAD_G:
	case BMG250_ENABLE:

		/* Partial read, then skip the data */
		if ((*idx + BMG250_FIFO_G_LENGTH) > dev->fifo->length) {
			/* Update the data index as complete */
			*idx = dev->fifo->length;
			break;
		}

		/*Unpack the data array into structure instance "gyro"*/
		unpack_gyro_data(&gyro[*gyro_idx], *idx, dev);

		/*Move the data index*/
		(*idx) = (*idx) + BMG250_FIFO_G_LENGTH;
		(*gyro_idx)++;
		break;
	default:
		break;
	}
}

/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data and store it in the instance of the structure bmg250_sensor_data.
 */
static void unpack_gyro_data(struct bmg250_sensor_data *gyro_data,
	uint16_t data_start_index,
	const struct bmg250_dev *dev)
{
	uint16_t data_lsb;
	uint16_t data_msb;

	/* Gyro raw x data */
	data_lsb = dev->fifo->data[data_start_index++];
	data_msb = dev->fifo->data[data_start_index++];
	gyro_data->x = (int16_t)(((uint16_t)data_msb << 8) | data_lsb);

	/* Gyro raw y data */
	data_lsb = dev->fifo->data[data_start_index++];
	data_msb = dev->fifo->data[data_start_index++];
	gyro_data->y = (int16_t)(((uint16_t)data_msb << 8) | data_lsb);

	/* Gyro raw z data */
	data_lsb = dev->fifo->data[data_start_index++];
	data_msb = dev->fifo->data[data_start_index++];
	gyro_data->z = (int16_t)(((uint16_t)data_msb << 8) | data_lsb);
}

/*!
 *  @brief This API is used to parse the gyro data from the
 *  FIFO data in header mode.
 */
static void extract_gyro_header_mode(struct bmg250_sensor_data *gyro_data,
	uint8_t *gyro_length,
	const struct bmg250_dev *dev)
{
	uint8_t frame_header = 0;
	uint16_t data_index;
	uint8_t gyro_index = 0;

	for (data_index = dev->fifo->gyro_byte_start_idx; data_index < dev->fifo->length;) {
		/* extracting Frame header */
		frame_header = (dev->fifo->data[data_index] & BMG250_FIFO_TAG_INTR_MASK);

		/*Index is moved to next byte where the data is starting*/
		data_index++;
		switch (frame_header) {
		/* Gyro frame */
		case BMG250_FIFO_HEAD_G:
			unpack_gyro_frame(gyro_data, &data_index, &gyro_index, frame_header, dev);
			break;

		/* Sensor time frame */
		case BMG250_FIFO_HEAD_SENSOR_TIME:
			unpack_sensortime_frame(&data_index, dev);
			break;

		/* Skip frame */
		case BMG250_FIFO_HEAD_SKIP_FRAME:
			unpack_skipped_frame(&data_index, dev);
			break;

		/* Input config frame */
		case BMG250_FIFO_HEAD_INPUT_CONFIG:
			move_next_frame(&data_index, 1, dev);
			break;
		case BMG250_FIFO_HEAD_OVER_READ:

			/* Update the data index as complete
			 * in case of over read */
			data_index = dev->fifo->length;
			break;
		default:
			break;
		}
	}

	/*Update number of gyro data read*/
	*gyro_length = gyro_index;

	/*Update the gyro frame index*/
	dev->fifo->gyro_byte_start_idx = data_index;
}

/*!
 *  @brief This API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev->fifo->sensor_time.
 */
static void unpack_sensortime_frame(uint16_t *data_index, const struct bmg250_dev *dev)
{
	uint32_t sensor_time_byte3 = 0;
	uint16_t sensor_time_byte2 = 0;
	uint8_t sensor_time_byte1 = 0;

	/* Partial read, then move the data index to last data*/
	if ((*data_index + BMG250_SENSOR_TIME_LENGTH) > dev->fifo->length) {
		/* Update the data index as complete*/
		*data_index = dev->fifo->length;
	} else {
		sensor_time_byte3 = dev->fifo->data[(*data_index) + 2] << 16;
		sensor_time_byte2 = dev->fifo->data[(*data_index) + 1] << 8;
		sensor_time_byte1 = dev->fifo->data[(*data_index)];

		/* Sensor time */
		dev->fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
		*data_index = (*data_index) + BMG250_SENSOR_TIME_LENGTH;
	}
}

/*!
 *  @brief This API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev->fifo->skipped_frame_count.
 */
static void unpack_skipped_frame(uint16_t *data_index, const struct bmg250_dev *dev)
{
	/* Partial read, then move the data index to last data */
	if (*data_index >= dev->fifo->length) {
		/* Update the data index as complete */
		*data_index = dev->fifo->length;
	} else {
		dev->fifo->skipped_frame_count = dev->fifo->data[*data_index];

		/* Move the data index */
		*data_index = (*data_index) + 1;
	}
}

/*!
 *  @brief This API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 */
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmg250_dev *dev)
{
	/*Partial read, then move the data index to last data*/
	if ((*data_index + current_frame_length) > dev->fifo->length) {
		/*Update the data index as complete*/
		*data_index = dev->fifo->length;
	} else {
		/*Move the data index to next frame*/
		*data_index = *data_index + current_frame_length;
	}
}

/*!
 *  @brief This API is used to enable the sensor to write the
 *  offset for data compensation
 */
static int8_t enable_gyro_offset(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	rslt = bmg250_get_regs(BMG250_OFFSET_EN_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		/* Enable the gyro offset enable bit */
		reg_data = BMG250_SET_BITS(reg_data, BMG250_OFFSET_ENABLE, BMG250_ENABLE);
		rslt = bmg250_set_regs(BMG250_OFFSET_EN_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}

/*!
 *  @brief This API is used to set the offset values for all the 3 axes of the sensor.
 */
static int8_t set_gyro_offset(const struct bmg250_offset *offset, const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t offset_msb;
	uint8_t temp_offset_msb;
	uint8_t data[3];

	/* Null-pointer check */
	rslt = null_ptr_check(dev);
	if (rslt == BMG250_OK) {
		/* Set the LSB of offsets for all 3 axes in the sensor */
		data[0] = BMG250_GET_BITS_POS_0(offset->x_offset, BMG250_GYRO_OFFSET_LSB);
		data[1] = BMG250_GET_BITS_POS_0(offset->y_offset, BMG250_GYRO_OFFSET_LSB);
		data[2] = BMG250_GET_BITS_POS_0(offset->z_offset, BMG250_GYRO_OFFSET_LSB);
		rslt = bmg250_set_regs(BMG250_OFFSET_X_ADDR, data, 3, dev);
		if (rslt == BMG250_OK) {
			/* Set the MSB of offsets in the sensor */
			rslt = bmg250_get_regs(BMG250_OFFSET_EN_ADDR, &offset_msb, 1, dev);
			if (rslt == BMG250_OK) {
				temp_offset_msb = BMG250_GET_BITS(offset->x_offset, BMG250_GYRO_OFFSET_MSB);
				offset_msb = offset_msb | temp_offset_msb;
				temp_offset_msb = BMG250_GET_BITS(offset->y_offset, BMG250_GYRO_OFFSET_MSB);
				offset_msb = offset_msb | (uint8_t)(temp_offset_msb << 2);
				temp_offset_msb = BMG250_GET_BITS(offset->z_offset, BMG250_GYRO_OFFSET_MSB);
				offset_msb = offset_msb | (uint8_t)(temp_offset_msb << 4);

				/* Set the offset value in 0x77 register */
				rslt = bmg250_set_regs(BMG250_OFFSET_EN_ADDR, &offset_msb, 1, dev);
			}
		}
	}

	return rslt;
}

/*!
 *  @brief This internal API is used to enable the SPI interface in the sensor
 */
static int8_t set_spi_enable(const struct bmg250_dev *dev)
{
	int8_t rslt;
	uint8_t reg_data;

	/* Read the NV_CONF register */
	rslt = bmg250_get_regs(BMG250_NV_CONF_ADDR, &reg_data, 1, dev);
	if (rslt == BMG250_OK) {
		/* Set the spi_en bit */
		reg_data = BMG250_SET_BITS_POS_0(reg_data, BMG250_SPI_EN, BMG250_ENABLE);

		/* Enable SPI in the sensor */
		rslt = bmg250_set_regs(BMG250_NV_CONF_ADDR, &reg_data, 1, dev);
	}

	return rslt;
}
