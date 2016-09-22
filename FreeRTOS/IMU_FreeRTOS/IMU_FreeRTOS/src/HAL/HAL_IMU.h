/*
 * HAL_IMU.h
 *
 * Created: 26/07/2016 21:42:46
 *  Author: Caio
 */ 


#ifndef HAL_IMU_H_
#define HAL_IMU_H_

#define NUM_AXIS			3

enum ADXL_Addr_Reg_t {
	ADXL_OffsetX	= 0x1E,
	ADXL_OffsetY,
	ADXL_OffsetZ,
	
	ADXL_BWRate		= 0x2C,
	ADXL_PowerCtl,
	ADXL_IntEnable,
	ADXL_InitMap,
	ADXL_InitSouce,
	ADXL_DataFormat,
	ADXL_DataX0,
	ADXL_DataX1,
	ADXL_DataY0,
	ADXL_DataY1,
	ADXL_DataZ0,
	ADXL_DataZ1,
	ADXL_FIFOCtl,
	ADXL_FIFOStatus
};

enum Rate_ADXL_t {
	BWrate0_10Hz,
	BWrate0_20Hz,
	BWrate0_39Hz,
	BWrate0_78Hz,
	BWrate1_56Hz,
	BWrate3_13Hz,
	BWrate6_25Hz,
	BWrate12_5Hz,
	BWrate25Hz,
	BWrate50Hz,
	BWrate100Hz,
	BWrate200Hz,
	BWrate400Hz,
	BWrate800Hz,
	BWrate1600Hz,
	BWrate3200Hz,
};

typedef struct rateADXL_t {
	uint8_t rateEnum;
	double	freq;
} rateADXL;

enum ADXL_Addr_Dev_t {
	ADXL_High		= 0x1D,
	ADXL_Low		= 0x53		//Standard Address
};

enum Axis_t {
	Axis_X,
	Axis_Y,
	Axis_Z
};

enum ITG_Addr_Reg_t {
	ITG_WhoAmI		= 0x00,
	
	ITG_SMPLRT_Div	= 0x15,
	ITG_DLPF_FS,
	ITG_Int_Cfg,
	
	ITG_Int_Status	= 0x1A,
	ITG_Temp_H,
	ITH_Temp_L,
	ITG_DataX1,
	ITG_DataX0,
	ITG_DataY1,
	ITG_DataY0,
	ITG_DataZ1,
	ITG_DataZ0,
	
	ITG_PWR_Mgm		= 0x3E
};

enum ITG_Addr_Dev_t {
	ITG_Low			= 0x68,		//Standard Address
	ITG_High
};

typedef enum ADXL_Addr_Reg_t	ADXL_Addr_Reg;
typedef enum ADXL_Addr_Dev_t	ADXL_Addr_Dev;
typedef enum Axis_t				Axis_Op;
typedef enum ITG_Addr_Reg_t		ITG_Addr_Reg;
typedef enum ITG_Addr_Dev_t		ITG_Addr_Dev;

// STANDARD IMU FUNCTIONS:
status_code_t configIMU(void);

//OFFSET FUNCTIONS:
Bool setOffsetAccel(Axis_Op ax, float offset);
float getOffsetAccel(Axis_Op ax);
Bool setOffsetGyro(Axis_Op ax, float offset);
float getOffsetGyro(Axis_Op ax);

// HIGH LEVEL FUNCITONS:
double getPureAngle(double *acel);
void getAllAcelValue(ADXL_Addr_Dev dev, double *acel);
void getAllGyroValue(ITG_Addr_Dev dev, double *gyro);
float get_gyro_value(Axis_Op axis, ITG_Addr_Dev dev);
float get_acel_value(Axis_Op axis, ADXL_Addr_Dev dev);

void clearInterruptADXL(ADXL_Addr_Dev dev);

#endif /* HAL_IMU_H_ */