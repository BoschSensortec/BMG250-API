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
* @file fifo.c
* @date 10/01/2020
* @version
*
*/

#include "bmg250.h"
#include "stdio.h"

/* Buffer size allocated to store raw FIFO data */
#define BMG250_FIFO_RAW_DATA_BUFFER_SIZE       UINT16_C(1000)

/* Length of data to be read from FIFO */
#define BMG250_FIFO_RAW_DATA_USER_LENGTH       UINT16_C(1000)

/* Number of Gyro frames to be extracted from FIFO */
#define BMG250_FIFO_EXTRACTED_DATA_FRAME_COUNT UINT8_C(200)

/* FIFO data filling delay */
#define BMG250_FIFO_CONFIG_READ_DELAY          UINT8_C(200)

/* Configuring, Reading FIFO and extraction of gyro data */
void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

int main(void)
{
    int8_t rslt;
    struct bmg250_dev gyro;
    struct bmg250_cfg gyro_cfg;
    uint16_t n_instance;
    uint8_t frames_needed;
    uint8_t fifo_data[BMG250_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };
    struct bmg250_fifo_frame fifo_frame;
    struct bmg250_sensor_data gyro_data[BMG250_FIFO_EXTRACTED_DATA_FRAME_COUNT];

    /* Map the delay function pointer with the function responsible for implementing the delay */
    gyro.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x68) & VDD for SECONDARY(0x69)) */
    gyro.dev_id = BMG250_I2C_ADDR;

    /* Select the interface mode as I2C */
    gyro.intf = BMG250_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    gyro.read = i2c_reg_read;
    gyro.write = i2c_reg_write;

    /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

    /*
     * gyro.dev_id = 0;
     * gyro.read = spi_reg_read;
     * gyro.write = spi_reg_write;
     * gyro.intf = BMG250_SPI_INTF;
     */
    rslt = bmg250_init(&gyro);
    print_rslt(" bmg250_init status", rslt);

    /* Setting the power mode as normal mode */
    gyro.power_mode = BMG250_GYRO_NORMAL_MODE;
    rslt = bmg250_set_power_mode(&gyro);
    print_rslt(" bmg250_set_power_mode status ", rslt);

    /* Read the set configuration from the sensor */
    rslt = bmg250_get_sensor_settings(&gyro_cfg, &gyro);
    print_rslt(" bmg250_get_sensor_settings status ", rslt);

    /* Gyro configuration settings */
    /* Output data rate in hertz */
    gyro_cfg.odr = BMG250_ODR_100HZ;

    /* Range(angular rate measurement rate)of the gyro in degrees per second */
    gyro_cfg.range = BMG250_RANGE_1000_DPS;

    /*Bandwidth settings for digital filter
     * For bandwidth = BMG250_BW_NORMAL_MODE, gyro data is sampled at equidistant points in the time defined by ORD.
     * For bandwidth = BMG250_BW_OSR2_MODE, both stages of digital filter are used & data is oversampled
     * with an oversampling rate of 2. The ODR has to be 2 times higher than that of the normal mode.
     * For bandwidth = BMG250_BW_OSR4_MODE, both stages of digital filter are used & data is oversampled
     * with an oversampling rate of 4. The ODR has to be 4 times higher than that of the normal mode.
     */
    gyro_cfg.bw = BMG250_BW_NORMAL_MODE;

    /* Set the gyro configurations */
    rslt = bmg250_set_sensor_settings(&gyro_cfg, &gyro);
    print_rslt(" bmg250_set_sensor_settings status ", rslt);

    /* Modify the FIFO buffer instance and link to the device instance */
    /* Mapping the buffer to store the fifo data */
    fifo_frame.data = fifo_data;

    /* Number of bytes to read from fifo */
    fifo_frame.length = BMG250_FIFO_RAW_DATA_USER_LENGTH;
    gyro.fifo = &fifo_frame;

    /* Disable other FIFO configuration settings in the sensor */
    rslt = bmg250_set_fifo_config(BMG250_FIFO_ALL_SETTING, BMG250_DISABLE, &gyro);
    print_rslt("bmg250_set_fifo_config", rslt);

    /* Enable required FIFO configuration settings in the sensor */
    rslt = bmg250_set_fifo_config(BMG250_FIFO_GYRO | BMG250_FIFO_HEADER | BMG250_FIFO_TIME, BMG250_ENABLE, &gyro);
    print_rslt("bmg250_set_fifo_config status", rslt);

    /* FIFO filling delay */
    gyro.delay_ms(BMG250_FIFO_CONFIG_READ_DELAY);

    /* Read FIFO data */
    printf("\n FIFO data bytes requested : %d \n", gyro.fifo->length);
    rslt = bmg250_get_fifo_data(&gyro);
    print_rslt("bmg250_get_fifo_data", rslt);
    printf("\n FIFO data bytes available : %d \n", gyro.fifo->length);
    for (n_instance = 0; n_instance < gyro.fifo->length; n_instance++)
    {
        /* Printing the FIFO data */
        printf("\n FIFO DATA [%d] : %x ", n_instance, gyro.fifo->data[n_instance]);
    }

    /* Extracting Gyro data */
    frames_needed = BMG250_FIFO_EXTRACTED_DATA_FRAME_COUNT;
    printf("\n FIFO gyro frames requested : %d \n", frames_needed);
    rslt = bmg250_extract_gyro(&gyro_data[0], &frames_needed, &gyro);
    printf("\n FIFO gyro frames extracted : %d \n", frames_needed);

    /* Print the extraxted gyro data frames */
    for (int8_t i = 0; i < frames_needed + 2; i++)
    {
        printf("\n FIFO DATA [%d] --> X:%d \t, Y:%d \t, Z:%d \n", i, gyro_data[i].x, gyro_data[i].y, gyro_data[i].z);
    }
    printf("\n gyro_byte_start_idx = %d \n", gyro.fifo->gyro_byte_start_idx);
    printf("\n skipped_frame_count = %d \n", gyro.fifo->skipped_frame_count);
    printf("\n sensor_time = %lu \n", gyro.fifo->sensor_time);

    return 0;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
    /* Implement the delay routine according to the target machine */
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C write routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C read routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the SPI write routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the SPI read routine according to the target machine. */
    return -1;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMG250_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMG250_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMG250_E_COM_FAIL)
        {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMG250_E_INVALID_TEMPERATURE)
        {
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMG250_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
