/*
 * IMU.h
 *
 * Created: 03/04/2016 16:45:52
 *  Author: Caio
 */ 


#ifndef IMU_H_
#define IMU_H_

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

enum ADXL_Addr_Dev_t {
	ADXL_High		= 0x1D,		//Standard Address
	ADXL_Low		= 0x53
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

typedef enum ADXL_Addr_Reg_t ADXL_Addr_Reg;
typedef enum ADXL_Addr_Dev_t ADXL_Addr_Dev;
typedef enum ITG_Addr_Reg_t ITG_Addr_Reg;
typedef enum ITG_Addr_Dev_t ITG_Addr_Dev;

uint8_t twi_init(void);

#endif /* IMU_H_ */