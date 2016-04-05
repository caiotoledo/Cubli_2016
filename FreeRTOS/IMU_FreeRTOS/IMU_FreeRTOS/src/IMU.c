/*
 * IMU.c
 *
 * Created: 03/04/2016 16:45:34
 *  Author: Caio
 */ 

#include "IMU.h"
#include "UART_Comm.h"

#define TWI_SPEED		400000	//400KHz Fast-Speed
#define TWI_BLOCK_TIME	(100/portTICK_RATE_MS)

freertos_twi_if freertos_twi = NULL;

static status_code_t configIMU(xSemaphoreHandle xse);
static uint8_t twi_init(void);
static status_code_t adxl_init(ADXL_Addr_Dev ADXL_Dev, xSemaphoreHandle xSem);
static status_code_t itg_init(ITG_Addr_Dev ITG_Dev, xSemaphoreHandle xSem);
static status_code_t itg_write(ITG_Addr_Dev itg_addr, uint8_t value, ITG_Addr_Reg itg_reg, xSemaphoreHandle xSem);
static status_code_t itg_read (ITG_Addr_Dev itg_addr, uint8_t *value, ITG_Addr_Reg itg_reg, uint8_t len, xSemaphoreHandle xSem);
static status_code_t adxl_write(ADXL_Addr_Dev adxl_addr, uint8_t value, ADXL_Addr_Reg adxl_reg, xSemaphoreHandle xSem);
static status_code_t adxl_read (ADXL_Addr_Dev adxl_addr, uint8_t *value, ADXL_Addr_Reg adxl_reg, uint8_t len, xSemaphoreHandle xSem);

void IMUTask(void *pvParameters){
	UNUSED(pvParameters);
	status_code_t status;
	
	xseIMU = xSemaphoreCreateMutex();
	
	status = configIMU(xseIMU);
	if (status != STATUS_OK){
		printf_mux("Error IMU!");
		LED_On(LED2_GPIO);
		vTaskDelete(NULL);
	}
	
	for (;;){
		vTaskDelay(TWI_BLOCK_TIME);
	}
}

static status_code_t configIMU(xSemaphoreHandle xse){
	status_code_t status;
	
	if (twi_init() != TWI_SUCCESS){
		printf_mux("TWI init Error!");
		return -1;
	}
	
	status = adxl_init(ADXL_Low, xse);
	if (status != STATUS_OK){
		printf_mux("ADXL init Error!");
		return status;
	}
	
	status = itg_init(ITG_Low, xse);
	if (status != STATUS_OK){
		printf_mux("ITG init Error!");
		return status;
	}
	
	return status;
}

static uint8_t twi_init(void){
	gpio_configure_pin(TWI0_DATA_GPIO, TWI0_DATA_FLAGS);
	gpio_configure_pin(TWI0_CLK_GPIO, TWI0_CLK_FLAGS);
	
	freertos_peripheral_options_t driver_options = {
		//Receiver Buffer
		NULL,
		//Size Receiver Buffer
		0,
		//Priority Interrupt
		0x0F,
		TWI_I2C_MASTER,
		( USE_TX_ACCESS_MUTEX | USE_TX_ACCESS_MUTEX )
	};
	
	freertos_twi = freertos_twi_master_init(TWI0, &driver_options);
	
	if (freertos_twi != NULL){
		uint8_t twi_flag = twi_set_speed(TWI0, TWI_SPEED, sysclk_get_cpu_hz());
		return twi_flag;
	}
	
	return (!TWI_SUCCESS);
}

static status_code_t adxl_init(ADXL_Addr_Dev ADXL_Dev, xSemaphoreHandle xSem){
	status_code_t result;
	
	result = adxl_write(ADXL_Dev, 0x0B, ADXL_DataFormat, xSem);
	if (result != STATUS_OK) return result;
	xSemaphoreTake(xSem, TWI_BLOCK_TIME);
	
	result = adxl_write(ADXL_Dev, 0x09, ADXL_BWRate, xSem);
	if (result != STATUS_OK) return result;
	xSemaphoreTake(xSem, TWI_BLOCK_TIME);
	
	result = adxl_write(ADXL_Dev, 0x08, ADXL_PowerCtl, xSem);
	if (result != STATUS_OK) return result;
	xSemaphoreTake(xSem, TWI_BLOCK_TIME);
	
	return result;
}

static status_code_t itg_init(ITG_Addr_Dev ITG_Dev, xSemaphoreHandle xSem){
	status_code_t result;
	
	result = itg_write(ITG_Dev, 0x1B, ITG_DLPF_FS, xSem);
	if (result != STATUS_OK) return result;
	xSemaphoreTake(xSem, TWI_BLOCK_TIME);
	
	result = itg_write(ITG_Dev, 0, ITG_SMPLRT_Div, xSem);
	if (result != STATUS_OK) return result;
	xSemaphoreTake(xSem, TWI_BLOCK_TIME);
	
	result = itg_write(ITG_Dev, 0x00, ITG_PWR_Mgm, xSem);
	if (result != STATUS_OK) return result;
	xSemaphoreTake(xSem, TWI_BLOCK_TIME);
	
	return result;
}

static status_code_t itg_write(ITG_Addr_Dev itg_addr, uint8_t value, ITG_Addr_Reg itg_reg, xSemaphoreHandle xSem){
	twi_packet_t tx = {
		.addr[0]		= ((uint8_t) itg_reg),
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ((uint8_t) itg_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_write_packet_async(freertos_twi, &tx, TWI_BLOCK_TIME, xSem);
	
	return result;
}

static status_code_t itg_read (ITG_Addr_Dev itg_addr, uint8_t *value, ITG_Addr_Reg itg_reg, uint8_t len, xSemaphoreHandle xSem){
	twi_packet_t rx = {
		.addr[0]		= ((uint8_t) itg_reg),
		.addr_length	= 1,
		.buffer			= value,
		.length			= len,
		.chip			= ((uint8_t) itg_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_read_packet_async(freertos_twi, &rx, TWI_BLOCK_TIME, xSem);
	
	return result;
}

static status_code_t adxl_write(ADXL_Addr_Dev adxl_addr, uint8_t value, ADXL_Addr_Reg adxl_reg, xSemaphoreHandle xSem){
	twi_packet_t tx = {
		.addr[0]		= ((uint8_t) adxl_reg),
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ((uint8_t) adxl_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_write_packet_async(freertos_twi, &tx, TWI_BLOCK_TIME, xSem);
	
	return result;
}

static status_code_t adxl_read (ADXL_Addr_Dev adxl_addr, uint8_t *value, ADXL_Addr_Reg adxl_reg, uint8_t len, xSemaphoreHandle xSem){
	twi_packet_t rx = {
		.addr[0]		= ((uint8_t) adxl_reg),
		.addr_length	= 1,
		.buffer			= value,
		.length			= len,
		.chip			= ((uint8_t) adxl_addr)
	};
	
	status_code_t result;
	
	result = freertos_twi_read_packet_async(freertos_twi, &rx, TWI_BLOCK_TIME, xSem);
	
	return result;
}