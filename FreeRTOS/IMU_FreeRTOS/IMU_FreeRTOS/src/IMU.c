/*
 * IMU.c
 *
 * Created: 03/04/2016 16:45:34
 *  Author: Caio
 */ 

#include "IMU.h"
#include "UART_Comm.h"
#include "LCD.h"
#include <string.h>

#define TWI_SPEED		400000	//400KHz Fast-Speed
#define TWI_BLOCK_TIME	(portMAX_DELAY)

#define CONST_ADXL		(3.9)
#define CONST_ITG		(14.375)

#define xQueueOverwrite(xQueue,pvItemToQueue)	xQueueReset(xQueue);xQueueSendToFront(xQueue,pvItemToQueue,(0))

freertos_twi_if freertos_twi;
char buffer[50];

xSemaphoreHandle xseIMU;
xSemaphoreHandle xSemIMUTimer;
xTimerHandle xTimerIMU;

static void vTimerIMU(void *pvParameters){
	xSemaphoreGive(xSemIMUTimer);
}

void IMUTask(void *pvParameters){
	UNUSED(pvParameters);
	status_code_t status;
	
	vSemaphoreCreateBinary(xSemIMUTimer);
	configASSERT(xSemIMUTimer);
	xSemaphoreTake(xSemIMUTimer, 0);
	xTimerIMU = xTimerCreate("TimerIMU", TWI_TASK_DELAY , pdTRUE, NULL, vTimerIMU);
	xTimerStart(xTimerIMU, 0);
	
	status = configIMU();
	if (status != STATUS_OK){
		printf_mux("Error IMU!");
		LED_On(LED2_GPIO);
		vTaskDelete(NULL);
	}
	
	float acel[3];
	float gyro[3];
	uint8_t i = 0;
	
	for (;;){
		xSemaphoreTake(xSemIMUTimer, portMAX_DELAY);
		
		memset(acel, 0, sizeof(acel));
		getAllAcelValue(ADXL_Low, acel);
		for (i = 0; i < NUM_AXIS; i++){
			xQueueOverwrite(xQueueAcel[i], (void * ) &acel[i]);
		}
		
		memset(gyro, 0, sizeof(gyro));		
		getAllGyroValue(ITG_Low, gyro);
		for (i = 0; i < NUM_AXIS; i++){
			xQueueOverwrite(xQueueGyro[i], (void * ) &gyro[i]);
		}
		
		printf_mux("Acel:\tX = %0.3f\tY = %0.3f\tZ = %0.3f", acel[0] , acel[1], acel[2]);
		printf_mux("Gyro:\tX = %0.3f\tY = %0.3f\tZ = %0.3f", gyro[0] , gyro[1], gyro[2]);
	}
}

static void getAllAcelValue(ADXL_Addr_Dev dev, float *acel){
	status_code_t result;
	memset(acel, (-16000), NUM_AXIS);
	uint8_t b[6] = {0};
	uint16_t adxl = 0;
	uint8_t i = 0;
	
	//Read all axis address:
	result = adxl_read(dev, b, ADXL_DataX0, sizeof(b));
	
	//If there is an error in read return with wrong values:
	if (result != STATUS_OK) return;
	
	//Convert each axis from two complements to float:
	for (i = 0; i < NUM_AXIS; i++){
		adxl = (uint16_t)( (b[(2*i)+1] << 8) | b[(2*i)] );
		if ( !(adxl & 0xFC00) ){
			acel[i] = CONST_ADXL * ((float) adxl);
		} else {
			adxl = ( ( (~adxl) +1 ) & 0x03FF);
			acel[i] = -(CONST_ADXL * ((float) adxl) );
		}
		acel[i] += offsetAcel[i];	//Apply Offset
	}
}

static void getAllGyroValue(ITG_Addr_Dev dev, float *gyro){
	status_code_t result;
	memset(gyro, (-16000), NUM_AXIS);
	uint8_t b[6] = {0};
	uint16_t itg = 0;
	uint8_t i = 0;
	
	//Read all axis address:
	result = itg_read(dev, b, ITG_DataX1, sizeof(b));
	
	//If there is an error in read return with wrong values:
	if (result != STATUS_OK) return;
	
	//Convert each axis from two complements to float:
	for (i = 0; i < NUM_AXIS; i++){
		itg = (uint16_t)( (b[(2*i)] << 8) | b[(2*i)+1] );
		if ( !(itg & 0x8000) ){
			gyro[i] = ((float) itg) / CONST_ITG;
			} else {
			itg = ( ( (~itg) +1 ) & 0x7FFF);
			gyro[i] = -( ((float) itg) / CONST_ITG );
		}
		gyro[i] += offsetGyro[i];	//Apply Offset
	}
}

static float get_gyro_value(Axis_Op axis, ITG_Addr_Dev dev){
	status_code_t result;
	float gyro_value = -800;
	uint16_t itg = 0;
	uint8_t b[2];
	memset(b, 0, sizeof(b));
	
	switch (axis)
	{
		case Axis_X:
			result = itg_read(dev, b, ITG_DataX1, sizeof(b));
			break;
		case Axis_Y:
			result = itg_read(dev, b, ITG_DataY1, sizeof(b));
			break;
		case Axis_Z:
			result = itg_read(dev, b, ITG_DataZ1, sizeof(b));
			break;
	}
	
	if (result != STATUS_OK){
		return gyro_value;
	}
	
	itg = (uint16_t)( (b[0] << 8) | b[1] );
	
	if ( !(itg & 0x8000) ){
		gyro_value = ((float)itg) / CONST_ITG;
	} else {
		itg = ( ( (~itg) +1 ) & 0x7FFF);
		gyro_value = -(((float)itg) / CONST_ITG);
	}
	
	return gyro_value;
}

static float get_acel_value(Axis_Op axis, ADXL_Addr_Dev dev){
	status_code_t result;
	float acel_value = -16000;
	uint16_t adxl = 0;
	uint8_t b[2];
	memset(b, 0, sizeof(b));
	
	signed portBASE_TYPE resultSem;
	
	switch (axis) {
		case Axis_X:
			result = adxl_read(dev, b, ADXL_DataX0, sizeof(b));
			break;
		case Axis_Y:
			result = adxl_read(dev, b, ADXL_DataY0, sizeof(b));
			break;
		case Axis_Z:
			result = adxl_read(dev, b, ADXL_DataZ0, sizeof(b));
			break;
	}
	
	if (result != STATUS_OK){
		return acel_value;
	}
	
	adxl = (uint16_t)( (b[1] << 8) | b[0] );
	
	if ( !(adxl & 0xFC00) ){
		acel_value = CONST_ADXL * ((float) adxl);
	} else {
		adxl = ( ( (~adxl) +1 ) & 0x03FF);
		acel_value = -(CONST_ADXL * ((float) adxl) );
	}
	
	return acel_value;
}

static status_code_t configIMU(){
	status_code_t status;
	
	if (twi_init() != TWI_SUCCESS){
		printf_mux("TWI init Error!");
		return -1;
	}
	
	status = adxl_init(ADXL_Low);
	if (status != STATUS_OK){
		printf_mux("ADXL init Error!");
		return status;
	}
	
	status = itg_init(ITG_Low);
	if (status != STATUS_OK){
		printf_mux("ITG init Error!");
		return status;
	}
	
	return status;
}

uint8_t twi_init(){	
	freertos_peripheral_options_t driver_options = {
		//Receiver Buffer
		NULL,
		//Size Receiver Buffer
		0,
		//Priority Interrupt
		(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY),
		TWI_I2C_MASTER,
		( USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE)
	};
	
	freertos_twi = freertos_twi_master_init(TWI0, &driver_options);
	
	configASSERT(freertos_twi);
	uint8_t twi_flag = twi_set_speed(TWI0, TWI_SPEED, sysclk_get_cpu_hz());
	return twi_flag;
}

static status_code_t adxl_init(ADXL_Addr_Dev ADXL_Dev){
	status_code_t result;
	
	//result = adxl_write(ADXL_Dev, 0x0B, ADXL_DataFormat, xSem);	//16g, 13-bit mode
	result = adxl_write(ADXL_Dev, 0x08, ADXL_DataFormat);	//2g, 10-bit mode
	if (result != STATUS_OK) return result;
	
	result = adxl_write(ADXL_Dev, 0x09, ADXL_BWRate);		//Sample rate = 50Hz = 20ms LPF = 25Hz
	if (result != STATUS_OK) return result;
	
	result = adxl_write(ADXL_Dev, 0x08, ADXL_PowerCtl);	//Start Measurement
	if (result != STATUS_OK) return result;
	
	return result;
}

static status_code_t itg_init(ITG_Addr_Dev ITG_Dev){
	status_code_t result;
	
	result = itg_write(ITG_Dev, 0x1B, ITG_DLPF_FS);		//2000º/s - Sample Rate: 1KHz - LPF: 42Hz
	if (result != STATUS_OK) return result;
	
	result = itg_write(ITG_Dev, 0, ITG_SMPLRT_Div);		//Fsample = 8KHz/(0 + 1) = 8KHz : 0,125ms
	if (result != STATUS_OK) return result;
	
	result = itg_write(ITG_Dev, 0x00, ITG_PWR_Mgm);		//Clock Source: Internal Oscillator
	if (result != STATUS_OK) return result;
	
	return result;
}

static status_code_t itg_write(ITG_Addr_Dev itg_addr, uint8_t value, ITG_Addr_Reg itg_reg){
	twi_packet_t tx = {
		.addr[0]		= ((uint8_t) itg_reg),
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ((uint8_t) itg_addr)
	};
	
	status_code_t result = STATUS_ERR_TIMEOUT;
	
	result = freertos_twi_write_packet(freertos_twi, &tx, TWI_BLOCK_TIME);
	
	return result;
}

static status_code_t itg_read (ITG_Addr_Dev itg_addr, uint8_t *value, ITG_Addr_Reg itg_reg, uint8_t len){
	twi_packet_t rx = {
		.addr[0]		= ((uint8_t) itg_reg),
		.addr_length	= 1,
		.buffer			= value,
		.length			= len,
		.chip			= ((uint8_t) itg_addr)
	};
	
	status_code_t result = STATUS_ERR_TIMEOUT;
	
	result = freertos_twi_read_packet(freertos_twi, &rx, TWI_BLOCK_TIME);
	
	return result;
}

static status_code_t adxl_write(ADXL_Addr_Dev adxl_addr, uint8_t value, ADXL_Addr_Reg adxl_reg){
	twi_packet_t tx = {
		.addr[0]		= ((uint8_t) adxl_reg),
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ((uint8_t) adxl_addr)
	};
	
	status_code_t result = STATUS_ERR_TIMEOUT;
	
	result = freertos_twi_write_packet(freertos_twi, &tx, TWI_BLOCK_TIME);
	
	return result;
}

static status_code_t adxl_read (ADXL_Addr_Dev adxl_addr, uint8_t *value, ADXL_Addr_Reg adxl_reg, uint8_t len){
	twi_packet_t rx = {
		.addr[0]		= ((uint8_t) adxl_reg),
		.addr_length	= 1,
		.buffer			= value,
		.length			= len,
		.chip			= ((uint8_t) adxl_addr)
	};
	
	status_code_t result = STATUS_ERR_TIMEOUT;

	result = freertos_twi_read_packet(freertos_twi, &rx, TWI_BLOCK_TIME);
	
	return result;
}