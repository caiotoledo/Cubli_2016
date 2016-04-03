/*
 * IMU.c
 *
 * Created: 03/04/2016 16:45:34
 *  Author: Caio
 */ 

#define TWI_SPEED		400000	//400KHz Fast-Speed
#define TWI_BLOCK_TIME	(100/portTICK_RATE_MS)

uint8_t twi_init(void){
	gpio_configure_pin(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
	gpio_configure_pin(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
	
	freertos_twi_if freertos_twi;
	
	freertos_peripheral_options_t driver_options = {
		//Receiver Buffer
		NULL,
		//Size Receiver Buffer
		0,
		//Priority Interrupt
		0x0F,
		TWI_I2C_MASTER,
		( USE_TX_ACCESS_MUTEX | USE_TX_ACCESS_MUTEX )
	}
	
	freertos_twi = freertos_twi_master_init(TWI0, driver_options);
	
	if (freertos_twi != NULL){
		uint8_t twi_flag = twi_set_speed(TWI0, TWI_SPEED, sysclk_get_cpu_hz());
		return twi_flag;
	} else {
		return (!TWI_SUCCESS);
	}
}

status_code_t itg_write(ITG_Addr_Dev itg_addr, uint8_t value, ITG_Addr_Reg itg_reg, xSemaphoreHandle xSem){
	twi_packet_t tx = {
		.addr[0]		= ((uint8_t) itg_reg),
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ((uint8_t) itg_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_write_packet_async(TWI0, &tx, TWI_BLOCK_TIME, xSem);
	
	return result;
}

status_code_t itg_read (ITG_Addr_Dev itg_addr, uint8_t *value, ITG_Addr_Reg itg_reg, uint8_t len, xSemaphoreHandle xSem){
	twi_packet_t rx = {
		.addr[0]		= ((uint8_t) itg_reg),
		.addr_length	= 1,
		.buffer			= value,
		.length			= len,
		.chip			= ((uint8_t) itg_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_read_packet_async(TWI0, &rx, TWI_BLOCK_TIME, xSem);
	
	return result;
}

status_code_t adxl_write(ADXL_Addr_Dev adxl_addr, uint8_t value, ADXL_Addr_Reg adxl_reg, xSemaphoreHandle xSem){
	twi_packet_t tx = {
		.addr[0]		= ((uint8_t) adxl_reg),
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ((uint8_t) adxl_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_write_packet_async(TWI0, &tx, TWI_BLOCK_TIME, xSem);
	
	return result;
}

status_code_t adxl_read (ADXL_Addr_Dev adxl_addr, uint8_t *value, ADXL_Addr_Reg adxl_reg, uint8_t len, xSemaphoreHandle xSem){
	twi_packet_t rx = {
		.addr[0]		= ((uint8_t) adxl_reg),
		.addr_length	= 1,
		.buffer			= value,
		.length			= len,
		.chip			= ((uint8_t) adxl_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_read_packet_async(TWI0, &rx, TWI_BLOCK_TIME, xSem);
	
	return result;
}