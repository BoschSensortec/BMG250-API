/*!
 *	@brief Example shows basic application of configuring the gyro sensor and reading the gyro sensor.
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

	/* Structure to set the gyro config */
	struct bmg250_cfg gyro_cfg;

	/* Structure to store the sensor data */
	struct bmg250_sensor_data gyro_data;

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
	   gyro.dev_id = 0;
	   gyro.read = spi_reg_read;
	   gyro.write = spi_reg_write;
	   gyro.intf = BMG250_SPI_INTF;
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
	 * For bandwidth = BMG250_BW_NORMAL_MODE, the gyro data is sampled at equidistant points in the time defined by the ORD.
	 * For bandwidth = BMG250_BW_OSR2_MODE, both stages of digital filter are used & data is oversampled
	   with an oversampling rate of 2. The ODR has to be 2 times higher than that of the normal mode.
	 * For bandwidth = BMG250_BW_OSR4_MODE, both stages of digital filter are used & data is oversampled
	   with an oversampling rate of 4. The ODR has to be 4 times higher than that of the normal mode.
	 */
	gyro_cfg.bw = BMG250_BW_NORMAL_MODE;

	/* Set the gyro configurations */
	rslt = bmg250_set_sensor_settings(&gyro_cfg, &gyro);
	print_rslt(" bmg250_set_sensor_settings status ", rslt);

	/* Reading the gyro data */
	while (1) {
		rslt = bmg250_get_sensor_data(BMG250_DATA_TIME_SEL, &gyro_data, &gyro);
		printf("Gyro data  X: %d \t Y: %d \t Z: %d \t Sensor-time : %d\n",
			gyro_data.x,
			gyro_data.y,
			gyro_data.z,
			gyro_data.sensortime);

		/* Delay of 10ms is added since ODR is 100Hz */
		gyro.delay_ms(10);
	}

	return 0;
}

/*!
 *	@brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *		"bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *	@param[in] period_ms  : the required wait time in milliseconds.
 *	@return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
	/* Implement the delay routine according to the target machine */
}

/*!
 *	@brief Function for writing the sensor's registers through I2C bus.
 *
 *	@param[in] i2c_addr : sensor I2C address.
 *	@param[in] reg_addr	: Register address.
 *	@param[in] reg_data	: Pointer to the data buffer whose value is to be written.
 *	@param[in] length	: No of bytes to write.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the I2C write routine according to the target machine. */
	return -1;
}

/*!
 *	@brief Function for reading the sensor's registers through I2C bus.
 *
 *	@param[in] i2c_addr : Sensor I2C address.
 *	@param[in] reg_addr	: Register address.
 *	@param[out] reg_data	: Pointer to the data buffer to store the read data.
 *	@param[in] length	: No of bytes to read.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the I2C read routine according to the target machine. */
	return -1;
}

/*!
 *	@brief Function for writing the sensor's registers through SPI bus.
 *
 *	@param[in] cs			: Chip select to enable the sensor.
 *	@param[in] reg_addr		: Register address.
 *	@param[in] reg_data	: Pointer to the data buffer whose data has to be written.
 *	@param[in] length		: No of bytes to write.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI write routine according to the target machine. */
	return -1;
}

/*!
 *	@brief Function for reading the sensor's registers through SPI bus.
 *
 *	@param[in] cs		: Chip select to enable the sensor.
 *	@param[in] reg_addr	: Register address.
 *	@param[out] reg_data	: Pointer to the data buffer to store the read data.
 *	@param[in] length	: No of bytes to read.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI read routine according to the target machine. */
	return -1;
}

/*!
 *	@brief Prints the execution status of the APIs.
 *
 *	@param[in] api_name	: name of the API whose execution status has to be printed.
 *	@param[in] rslt		: error code returned by the API whose execution status has to be printed.
 *
 *	@return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
	if (rslt != BMG250_OK) {
		printf("%s\t", api_name);
		if (rslt == BMG250_E_NULL_PTR) {
			printf("Error [%d] : Null pointer error\r\n", rslt);
		} else if (rslt == BMG250_E_COM_FAIL) {
			printf("Error [%d] : Bus communication failed\r\n", rslt);
		} else if (rslt == BMG250_E_INVALID_TEMPERATURE) {
			printf("Error [%d] : Invalid Temperature\r\n", rslt);
		} else if (rslt == BMG250_E_DEV_NOT_FOUND) {
			printf("Error [%d] : Device not found\r\n", rslt);
		} else {
			/* For more error codes refer "*_defs.h" */
			printf("Error [%d] : Unknown error code\r\n", rslt);
		}
	}
}
