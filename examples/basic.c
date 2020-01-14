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
* @file basic.c
* @date 10/01/2020
* @version
*
*/

#include "bmg250.h"
#include "stdio.h"

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

    return 0;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as"bmg250_soft_reset",
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
