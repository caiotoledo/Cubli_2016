/*
 * HAL_IMU_MPU6050.h
 *
 * Created: 26/07/2016 21:42:46
 *  Author: Caio
 */ 


#ifndef HAL_IMU_MPU6050_H_
#define HAL_IMU_MPU6050_H_

#define NUM_AXIS			3

enum IMU_Addr_Reg_t {
	IMU_SMPLRT_DIV		= 0x19,
	
	IMU_CONFIG			= 0x1A,
	IMU_GYRO_CONFIG,
	IMU_ACCEL_CONFIG,
	
	IMU_INT_ENABLE		= 0x38,
	
	IMU_ACCEL_XOUT_H	= 0x3B,
	IMU_ACCEL_XOUT_L,
	IMU_ACCEL_YOUT_H,
	IMU_ACCEL_YOUT_L,
	IMU_ACCEL_ZOUT_H,
	IMU_ACCEL_ZOUT_L,
	IMU_TEMP_OUT_H,
	IMU_TEMP_OUT_L,
	IMU_GYRO_XOUT_H,
	IMU_GYRO_XOUT_L,
	IMU_GYRO_YOUT_H,
	IMU_GYRO_YOUT_L,
	IMU_GYRO_ZOUT_H,
	IMU_GYRO_ZOUT_L,
	
	IMU_USER_CTRL		= 0x6A,
	IMU_PWR_MGMT_1,
	IMU_PWR_MGMT_2,
	
	IMU_WHO_AM_I		= 0x75
};

enum IMU_Addr_Dev_t {
	IMU_Low				= 0x68,
	IMU_High,
};

enum Axis_t {
	Axis_X,
	Axis_Y,
	Axis_Z
};

typedef enum IMU_Addr_Reg_t	IMU_Addr_Reg;
typedef enum IMU_Addr_Dev_t	IMU_Addr_Dev;
typedef enum Axis_t				Axis_Op;

// STANDARD IMU FUNCTIONS:
status_code_t configIMU(void);
uint32_t imu_probe(IMU_Addr_Dev dev);

// CALIBRATION FUNCTION:
uint32_t runIMUCalibration(IMU_Addr_Dev dev, Axis_Op gAxis, Bool positiveG);

//OFFSET FUNCTIONS:
Bool setOffsetAccel(Axis_Op ax, float offset);
float getOffsetAccel(Axis_Op ax);
Bool setOffsetGyro(Axis_Op ax, float offset);
float getOffsetGyro(Axis_Op ax);
//NEW OFFSET FUNCTIONS:
Bool setOffsetAccelIMU(IMU_Addr_Dev dev, Axis_Op ax, float offset);
float getOffsetAccelIMU(IMU_Addr_Dev dev, Axis_Op ax);
Bool setOffsetGyroIMU(IMU_Addr_Dev dev, Axis_Op ax, float offset);
float getOffsetGyroIMU(IMU_Addr_Dev dev, Axis_Op ax);

// HIGH LEVEL FUNCITONS:
double getPureAngle(double *acel);
void getAllAcelValue(IMU_Addr_Dev dev, double *acel);
void getAllGyroValue(IMU_Addr_Dev dev, double *gyro);
float get_gyro_value(Axis_Op axis, IMU_Addr_Dev dev);
float get_acel_value(Axis_Op axis, IMU_Addr_Dev dev);

#endif /* HAL_IMU_MPU6050_H_ */