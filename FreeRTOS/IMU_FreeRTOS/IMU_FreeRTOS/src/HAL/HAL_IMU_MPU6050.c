/*
 * HAL_IMU.c
 *
 * Created: 26/07/2016 21:42:36
 *  Author: Caio
 */ 

#include <asf.h>
#include <string.h>
#include <math.h>
#include "HAL_IMU_MPU6050.h"
#include "HAL/HAL_UART.h"

#define TWI_SPEED		400000	//400KHz Fast-Speed
#define TWI_BLOCK_TIME	(portMAX_DELAY)

#define ACEL_OFFSET_X		0//(-57.4)//(-37.0)
#define ACEL_OFFSET_Y		0//(7.0)
#define ACEL_OFFSET_Z		0//(-5.0)//(-26.0)

#define GYRO_OFFSET_X		(0.0)
#define GYRO_OFFSET_Y		(0.0)
#define GYRO_OFFSET_Z		(0.0)//(1.25)//(1.0)

#define CONST_ACCEL			(16.384)
#define CONST_GYRO			(131)

static uint8_t twi_init(void);
static uint8_t imu_init(IMU_Addr_Dev IMU_Dev);
static status_code_t imu_write(IMU_Addr_Dev imu_addr, uint8_t value, IMU_Addr_Reg imu_reg);
static status_code_t imu_read (IMU_Addr_Dev imu_addr, uint8_t *value, IMU_Addr_Reg imu_reg, uint8_t len);

static double offsetAcelIMU[2][NUM_AXIS] = {
	{ ACEL_OFFSET_X, ACEL_OFFSET_Y, ACEL_OFFSET_Z },
	{ ACEL_OFFSET_X, ACEL_OFFSET_Y, ACEL_OFFSET_Z }
};

static double offsetGyroIMU[2][NUM_AXIS] = {
	{ GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z },
	{ GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z }
};

static float offsetAcel[] = {
	ACEL_OFFSET_X,
	ACEL_OFFSET_Y,
	ACEL_OFFSET_Z
};

static float offsetGyro[] = {
	GYRO_OFFSET_X,
	GYRO_OFFSET_Y,
	GYRO_OFFSET_Z
};

freertos_twi_if freertos_twi;

Bool setOffsetAccelIMU(IMU_Addr_Dev dev, Axis_Op ax, float offset){
	Bool flag = false;
	
	/* Defines the device for offset array */
	uint8_t IMUdev = IMUADDR_TO_NUM(dev);
	
	if ( (offset > (-2000.0)) && (offset < (2000.0)) ) {
		offsetAcelIMU[IMUdev][ax] = offset;
		flag = true;
	}
	
	return flag;
}

Bool setOffsetAccel(Axis_Op ax, float offset){
	Bool flag = false;
	if ( (offset > (-2000.0)) && (offset < (2000.0)) ) {
		offsetAcel[ax] = offset;
		flag = true;
	}
	return flag;
}

float getOffsetAccelIMU(IMU_Addr_Dev dev, Axis_Op ax){
	
	/* Defines the device for offset array */
	uint8_t IMUdev = IMUADDR_TO_NUM(dev);
	
	return offsetAcelIMU[IMUdev][ax];
}

float getOffsetAccel(Axis_Op ax){
	return offsetAcel[ax];
}

Bool setOffsetGyroIMU(IMU_Addr_Dev dev, Axis_Op ax, float offset){
	Bool flag = false;
	
	/* Defines the device for offset array */
	uint8_t IMUdev = IMUADDR_TO_NUM(dev);
	
	if ( (offset > (-2000.0)) && (offset < (2000.0)) ) {
		offsetGyroIMU[IMUdev][ax] = offset;
		flag = true;
	}
	
	return flag;
}

Bool setOffsetGyro(Axis_Op ax, float offset){
	Bool flag = false;
	if ( (offset > (-2000.0)) && (offset < (2000.0)) ) {
		offsetGyro[ax] = offset;
		flag = true;
	}
	return flag;
}

float getOffsetGyroIMU(IMU_Addr_Dev dev, Axis_Op ax){
	
	/* Defines the device for offset array */
	uint8_t IMUdev = IMUADDR_TO_NUM(dev);
	
	return offsetGyroIMU[IMUdev][ax];
}

float getOffsetGyro(Axis_Op ax){
	return offsetGyro[ax];
}

uint32_t sampleIMU(IMU_Addr_Dev dev, double *acel, double *gyro){
	uint32_t status_imu;
	
	memset(acel, 0, NUM_AXIS);
	memset(gyro, 0, NUM_AXIS);
	
	status_imu = imu_probe(dev);
	if (status_imu != TWI_SUCCESS) {
		return status_imu;
	}
	
	getAllAcelValue(dev, acel);
	getAllGyroValue(dev, gyro);
	
	return status_imu;
}

/************************************************************************/
/* Based on "The Cubli: A Cube that can Jump Up and Balance" III - a    */
/************************************************************************/
#define R1		300.0
#define R2		1.0
const float ratioRelation = (R1/R2);
double getPureAngleTwoIMU(double *acelLow, double *acelHigh){
	double mx,my;
	
	mx = acelHigh[Axis_X] - ratioRelation * acelLow[Axis_X];
	my = acelHigh[Axis_Y] - ratioRelation * acelLow[Axis_Y];
	
	double angle = (atan(-mx/my)) * (180.0/M_PI);
	
	return angle;
}

double getPureAngle(double *acel){
	double angle = 0.0;
		
	/*angle = acos( (acel[Axis_X]) / sqrt(acel[Axis_Y]*acel[Axis_Y] + acel[Axis_Z]*acel[Axis_Z] + acel[Axis_X]*acel[Axis_X] ) ) * (180.0/M_PI);
	if (acel[Axis_Y] < 0){
		angle = - angle;
	}*/
	
	angle = acos( (acel[Axis_Y]) / sqrt(acel[Axis_Y]*acel[Axis_Y] + acel[Axis_Z]*acel[Axis_Z] + acel[Axis_X]*acel[Axis_X] ) ) * (180.0/M_PI);
	if (acel[Axis_X] < 0){
		angle = - angle;
	}
	
	//angle = (sin( acel[Axis_Y]/ acel[Axis_X] ) * (180.0/M_PI));
	
	return angle;
}

void getAllAcelValue(IMU_Addr_Dev dev, double *acel){
	status_code_t result;
	uint8_t b[6] = {0};
	uint16_t raw_accel = 0;
	uint8_t i = 0;
	
	/* Reset Buffer: */
	for (i = 0; i < NUM_AXIS; i++) {
		acel[i] = -16000.0;
	}
	
	/* Check if the dev exists: */
	if (imu_probe(dev) != TWI_SUCCESS){
		return;
	}
	
	//Read all axis address:
	result = imu_read(dev, b, IMU_ACCEL_XOUT_H, sizeof(b));
	
	//If there is an error in read return with wrong values:
	if (result != STATUS_OK) return;
	
	/* Defines the device for offset array */
	uint8_t IMUdev = IMUADDR_TO_NUM(dev);
	
	//Convert each axis from two complements to float:
	for (i = 0; i < NUM_AXIS; i++){
		raw_accel = (uint16_t)( (b[(2*i)] << 8) | b[(2*i)+1] );
		if ( !(raw_accel & 0x8000) ){
			acel[i] = ((float) raw_accel) / CONST_ACCEL;
			} else {
			raw_accel = ( ( (~raw_accel) +1 ) & 0x7FFF);
			acel[i] = -(((float) raw_accel) / CONST_ACCEL );
		}
		acel[i] += offsetAcelIMU[IMUdev][i];	//Apply Offset
	}
}

void getAllGyroValue(IMU_Addr_Dev dev, double *gyro){
	status_code_t result;
	uint8_t b[6] = {0};
	uint16_t itg = 0;
	uint8_t i = 0;
	
	/* Reset Buffer: */
	for (i = 0; i < NUM_AXIS; i++) {
		gyro[i] = -16000.0;
	}
	
	/* Check if the dev exists: */
	if (imu_probe(dev) != TWI_SUCCESS){
		return;
	}
	
	//Read all axis address:
	result = imu_read(dev, b, IMU_GYRO_XOUT_H, sizeof(b));
	
	//If there is an error in read return with wrong values:
	if (result != STATUS_OK) return;
	
	/* Defines the device for offset array */
	uint8_t IMUdev = IMUADDR_TO_NUM(dev);
	
	//Convert each axis from two complements to float:
	for (i = 0; i < NUM_AXIS; i++){
		itg = (uint16_t)( (b[(2*i)] << 8) | b[(2*i)+1] );
		if ( !(itg & 0x8000) ){
			gyro[i] = ((float) itg) / CONST_GYRO;
			} else {
			itg = ( ( (~itg) +1 ) & 0x7FFF);
			gyro[i] = -( ((float) itg) / CONST_GYRO );
		}
		gyro[i] += offsetGyroIMU[IMUdev][i];	//Apply Offset
		
		/* Gyro Value is opposite value from Accelerometer */
		gyro[i] = - gyro[i];
	}
}

float get_gyro_value(Axis_Op axis, IMU_Addr_Dev dev){
	status_code_t result = ERR_IO_ERROR;
	float gyro_value = -16000;
	uint16_t itg = 0;
	uint8_t b[2];
	memset(b, 0, sizeof(b));
	
	/* Check if the dev exists: */
	if (imu_probe(dev) != TWI_SUCCESS){
		return gyro_value;
	}
	
	switch (axis)
	{
		case Axis_X:
		result = imu_read(dev, b, IMU_GYRO_XOUT_H, sizeof(b));
		break;
		case Axis_Y:
		result = imu_read(dev, b, IMU_GYRO_YOUT_H, sizeof(b));
		break;
		case Axis_Z:
		result = imu_read(dev, b, IMU_GYRO_ZOUT_H, sizeof(b));
		break;
	}
	
	if (result != STATUS_OK){
		return gyro_value;
	}
	
	itg = (uint16_t)( (b[0] << 8) | b[1] );
	
	if ( !(itg & 0x8000) ){
		gyro_value = ((float)itg) / CONST_GYRO;
	} else {
		itg = ( ( (~itg) +1 ) & 0x7FFF);
		gyro_value = -(((float)itg) / CONST_GYRO);
	}
	
	/* Gyro Value is opposite value from Accelerometer */
	return (- gyro_value);
}

float get_acel_value(Axis_Op axis, IMU_Addr_Dev dev){
	status_code_t result = ERR_IO_ERROR;
	float acel_value = -16000;
	uint16_t adxl = 0;
	uint8_t b[2];
	memset(b, 0, sizeof(b));
	
	/* Check if the dev exists: */
	if (imu_probe(dev) != TWI_SUCCESS){
		return acel_value;
	}
	
	switch (axis) {
		case Axis_X:
		result = imu_read(dev, b, IMU_ACCEL_XOUT_H, sizeof(b));
		break;
		case Axis_Y:
		result = imu_read(dev, b, IMU_ACCEL_YOUT_H, sizeof(b));
		break;
		case Axis_Z:
		result = imu_read(dev, b, IMU_ACCEL_ZOUT_H, sizeof(b));
		break;
	}
	
	if (result != STATUS_OK){
		return acel_value;
	}
	
	adxl = (uint16_t)( (b[0] << 8) | b[1] );
	
	if ( !(adxl & 0x8000) ){
		acel_value = CONST_ACCEL * ((float) adxl);
	} else {
		adxl = ( ( (~adxl) +1 ) & 0x7FFF);
		acel_value = -(CONST_ACCEL * ((float) adxl) );
	}
	
	return acel_value;
}

#define TOTAL_SAMPLE_CALIBRATION		100
#define DELAY_BETWEEN_SAMPLES			(100/portTICK_RATE_MS)
uint32_t runIMUCalibration(IMU_Addr_Dev dev, Axis_Op gAxis, Bool positiveG){
	double acel[3] = {0};
	double gyro[3] = {0};
	double sumAcel[3] = {0};
	double sumGyro[3] = {0};
	double gForce[3] = {0};
	uint32_t i, j;
	
	/* Check if IMU exists */
	uint32_t result = imu_probe(dev);
	if (result != TWI_SUCCESS){
		return result;
	}
	
	/* It define what axis from IMU is faced to G Force: */
	gForce[gAxis] = (positiveG ? (1.0) : (-1.0) ) * 1000;
	
	/* Define Zero for all Offsets to not be use in getAllAcelValue and getAllGyroValue */
	uint8_t IMUdev = IMUADDR_TO_NUM(dev);
	for (j = 0; j < NUM_AXIS; j++)
	{
		offsetAcelIMU[IMUdev][j] = 0;
		offsetGyroIMU[IMUdev][j] = 0;
	}
	
	/* Here's where it sample: */
	for (i = 0; i < TOTAL_SAMPLE_CALIBRATION; i++) {
		getAllAcelValue(dev, acel);
		getAllGyroValue(dev, gyro);
		
		for (j = 0; j < NUM_AXIS; j++) {
			sumAcel[j] += acel[j];
			sumGyro[j] += gyro[j];
		}
		
		vTaskDelay(DELAY_BETWEEN_SAMPLES);
	}
	
	/* Update all Offsets */
	for (j = 0; j < NUM_AXIS; j++)
	{
		offsetAcelIMU[IMUdev][j] = (sumAcel[j]/TOTAL_SAMPLE_CALIBRATION) - gForce[j];
		offsetGyroIMU[IMUdev][j] = (sumGyro[j]/TOTAL_SAMPLE_CALIBRATION);
	}
	
	return result;
}

uint32_t imu_probe(IMU_Addr_Dev dev){
	return imu_write(dev, 0, 0);
	//return twi_probe(TWI0, dev);
}

status_code_t configIMU(void){
	status_code_t status = ERR_IO_ERROR;
	uint32_t probeState = TWI_NO_CHIP_FOUND;
	
	if (twi_init() != TWI_SUCCESS){
		printf_mux("TWI Init Error!");
		return ERR_IO_ERROR;
	}
	
	probeState = imu_probe(IMU_Low);
	if (probeState == TWI_SUCCESS) {
		
		status = imu_init(IMU_Low);
		if (status != STATUS_OK){
			printf_mux("MPU6050 Low Init Error!\n");
		}
		
	} else {
		printf_mux("IMU Low not Found!\n");
	}
	
	probeState = imu_probe(IMU_High);
	if (probeState == TWI_SUCCESS) {
		
		status = imu_init(IMU_High);
		if (status != STATUS_OK){
			printf_mux("MPU6050 High Init Error!\n");
		}
		
	} else {
		printf_mux("IMU High not Found!\n");
	}
	
	return status;
}

static uint8_t twi_init(void){
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

static uint8_t imu_init(IMU_Addr_Dev IMU_Dev){
	status_code_t result = STATUS_OK;	
	
	/*
	*	Define Sample Rate:
	*	Sample Rate = Gyroscope Output Rate/(1 + SMPLRT_DIV)
	*	Gyroscope Output Rate = 8 KHz
	*/
	result = imu_write(IMU_Dev, 0, IMU_SMPLRT_DIV);	//Sample Rate = 8 KHz
	if (result != STATUS_OK) return result;
	
	/*
	*	EXT_SYNC_SET- External Frame Synchronication
	*	DLPF_CFG	- Digital Low Pass Filter
	*		Accel	- BW 260Hz / Delay: 0 ms
	*		Gyro	- BW 256Hz / Delay: 0.98 ms
	*/
	result = imu_write(IMU_Dev, 0, IMU_CONFIG);
	if (result != STATUS_OK) return result;
	
	/*
	*	FS_SEL	- Full Scale for Gyroscope
	*		FS_SEL: 0 = +-250 °/s
	*/
	result = imu_write(IMU_Dev, 0, IMU_GYRO_CONFIG);
	if (result != STATUS_OK) return result;
	
	/*
	*	AFS_SEL	- Full Scale for Accelerometer
	*		AFS_SEL: 0 = +-2g
	*/
	result = imu_write(IMU_Dev, 0, IMU_ACCEL_CONFIG);
	if (result != STATUS_OK) return result;
	
	/*
	*	Interrupt Enable
	*		All interrupt disable
	*/
	result = imu_write(IMU_Dev, 0, IMU_INT_ENABLE);
	if (result != STATUS_OK) return result;
	
	result = imu_write(IMU_Dev, 0, IMU_USER_CTRL);
	if (result != STATUS_OK) return result;
	
	/*
	*	Power Management 1
	*		Disable Temperature sensor
	*		PLL with Z axis gyroscope reference
	*		Enable IMU
	*/
	//result = imu_write(IMU_Dev, 0x0B, IMU_PWR_MGMT_1);
	result = imu_write(IMU_Dev, 0, IMU_PWR_MGMT_1);
	if (result != STATUS_OK) return result;
	
	return result;
}

static status_code_t imu_write(IMU_Addr_Dev imu_addr, uint8_t value, IMU_Addr_Reg imu_reg){
	twi_packet_t tx = {
		.addr[0]		= ((uint8_t) imu_reg),
		.addr_length	= 1,
		.buffer			= &value,
		.length			= 1,
		.chip			= ((uint8_t) imu_addr)
	};
	
	status_code_t result = STATUS_ERR_TIMEOUT;
	
	result = freertos_twi_write_packet(freertos_twi, &tx, TWI_BLOCK_TIME);
	
	return result;
}

static status_code_t imu_read (IMU_Addr_Dev imu_addr, uint8_t *value, IMU_Addr_Reg imu_reg, uint8_t len){
	twi_packet_t rx = {
		.addr[0]		= ((uint8_t) imu_reg),
		.addr_length	= 1,
		.buffer			= value,
		.length			= len,
		.chip			= ((uint8_t) imu_addr)
	};
	
	status_code_t result = STATUS_ERR_TIMEOUT;
	
	result = freertos_twi_read_packet(freertos_twi, &rx, TWI_BLOCK_TIME);
	
	return result;
}