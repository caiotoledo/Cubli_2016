/*
 * IMU.c
 *
 * Created: 03/04/2016 16:45:34
 *  Author: Caio
 */ 

#include "IMU.h"
#include "UART_Comm.h"
#include "LCD.h"
#include "KalmanFilter.h"
#include <string.h>
#include <math.h>

#define TWI_SPEED		400000	//400KHz Fast-Speed
#define TWI_BLOCK_TIME	(portMAX_DELAY)

#define CONST_ADXL		(3.9)
#define CONST_ITG		(14.375)

#define xQueueOverwrite(xQueue,pvItemToQueue)	xQueueReset(xQueue);xQueueSendToFront(xQueue,pvItemToQueue,(0))

#define ALPHA			0.7143//0.8333//0.9091

freertos_twi_if freertos_twi;
char buffer[50];

xSemaphoreHandle xseIMU;
xSemaphoreHandle xSemIMUInt;
xTimerHandle xTimerIMU;

//uint8_t configBWRate = BWrate6_25Hz;
uint8_t configBWRate = BWrate100Hz;
uint32_t lastTickCounter = 0;
double dt = (((double)TWI_TASK_DELAY)/ ((double)configTICK_RATE_HZ));

static KalmanConst kalmanC;

static void vTimerIMU(void *pvParameters){
	xSemaphoreGive(xSemIMUInt);
}

void intpin_handler(uint32_t id, uint32_t mask){
	//dt = (double)(g_tickCounter - lastTickCounter)/configTICK_RATE_HZ;
	lastTickCounter = g_tickCounter;
	xSemaphoreGiveFromISR(xSemIMUInt, NULL);
}

void IMUTask(void *pvParameters){
	UNUSED(pvParameters);
	status_code_t status;
	
	vSemaphoreCreateBinary(xseIMUValues);
	configASSERT(xseIMUValues);
	xSemaphoreTake(xseIMUValues, 0);
	
	vSemaphoreCreateBinary(xSemIMUInt);
	configASSERT(xSemIMUInt);
	xSemaphoreTake(xSemIMUInt, 0);
	
#ifndef INT_PIN
	//Timer Task:
	xTimerIMU = xTimerCreate("TimerIMU", TWI_TASK_DELAY , pdTRUE, NULL, vTimerIMU);
	xTimerStart(xTimerIMU, 0);
#endif
	
	status = configIMU();
	lastTickCounter = g_tickCounter;
	if (status != STATUS_OK){
		printf_mux("Error IMU!");
		LED_On(LED2_GPIO);
		vTaskDelete(NULL);
	}
	
	double acel[3];
	double gyro[3];
	double anglePure = 0;
	double angleComplFilter = initComplFilter(ADXL_Low);
	double angleKalman = angleComplFilter;
	uint8_t i = 0;
	
	//Init Kalman Constants:
	kalmanC.angleInit	=	angleKalman;
	kalmanC.bias		=	0;
	kalmanC.Qangle		=	0.001;
	kalmanC.Qbias		=	0.003;
	kalmanC.Rmeasure	=	0.03;
	kalmanC.P[0][0]		=	0;
	kalmanC.P[0][1]		=	0;
	kalmanC.P[1][0]		=	0;
	kalmanC.P[1][1]		=	0;
	
	initKalman(&kalmanC);
	
	for (;;){
		xSemaphoreTake(xSemIMUInt, portMAX_DELAY);
		
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
		
		anglePure = getPureAngle(acel);
		xQueueOverwrite(xQueueAngle[0], (void * ) &anglePure);
		getComplFilterAngle(&angleComplFilter, acel, gyro);
		xQueueOverwrite(xQueueAngle[1], (void * ) &angleComplFilter);
		angleKalman = getKalmanAngle(anglePure, gyro[Axis_Z], dt);
		xQueueOverwrite(xQueueAngle[2], (void * ) &angleKalman);
		
		//Give Semaphore to UART Transfer:
		xSemaphoreGive(xseIMUValues);
	}
}

static double getPureAngle(double *acel){
	double angle = 0.0;
	
	angle = sin(-acel[Axis_X]/acel[Axis_Y]) * (180.0/M_PI);
	
	return angle;
}

static double initComplFilter(ADXL_Addr_Dev dev){
	double acelInit[3];
	getAllAcelValue(dev, acelInit);
	return getPureAngle(acelInit);
}

static void getComplFilterAngle(double *angle, double *acel, double *gyro){
	double angle_measure;
	
	angle_measure = getPureAngle(acel);
	
	*angle = (*angle + (gyro[Axis_Z]*dt) )*ALPHA + (1-ALPHA)*angle_measure;
}

static void getAllAcelValue(ADXL_Addr_Dev dev, double *acel){
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

static void getAllGyroValue(ITG_Addr_Dev dev, double *gyro){
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
	
	result = adxl_write(ADXL_Dev, 0x08, ADXL_DataFormat);	//2g, 10-bit mode
	if (result != STATUS_OK) return result;
	
	result = adxl_write(ADXL_Dev, configBWRate, ADXL_BWRate);
	if (result != STATUS_OK) return result;
	
	result = adxl_write(ADXL_Dev, 0x80, ADXL_IntEnable);	//Interrupt EN = Data Ready
	if (result != STATUS_OK) return result;
	
	result = adxl_write(ADXL_Dev, 0x7F, ADXL_InitMap);		//Interrupt Map = DataReady: INT1 / Others: INT2
	if (result != STATUS_OK) return result;
	
	clearInterruptADXL(ADXL_Dev);
	
	result = adxl_write(ADXL_Dev, 0x08, ADXL_PowerCtl);		//Start Measurement
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

static void clearInterruptADXL(ADXL_Addr_Dev dev){
	uint8_t b;
	adxl_read(dev, &b, ADXL_DataX0, 1);
}