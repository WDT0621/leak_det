/** 
* ADIS16470附属头文件
*/  
#ifndef _ADXIS_H
#define _ADXIS_H

#include "stm32f4xx_hal.h"
// User Register Memory Map from Table 6
#define FLASH_CNT   	0x00  //Flash memory write count
#define DIAG_STAT   	0x02  //Diagnostic and operational status
#define X_GYRO_LOW  	0x04  //X-axis gyroscope output, lower word
#define X_GYRO_OUT  	0x06  //X-axis gyroscope output, upper word
#define Y_GYRO_LOW  	0x08  //Y-axis gyroscope output, lower word
#define Y_GYRO_OUT  	0x0A  //Y-axis gyroscope output, upper word
#define Z_GYRO_LOW  	0x0C  //Z-axis gyroscope output, lower word
#define Z_GYRO_OUT  	0x0E  //Z-axis gyroscope output, upper word
#define X_ACCL_LOW  	0x10  //X-axis accelerometer output, lower word
#define X_ACCL_OUT  	0x12  //X-axis accelerometer output, upper word
#define Y_ACCL_LOW  	0x14  //Y-axis accelerometer output, lower word
#define Y_ACCL_OUT  	0x16  //Y-axis accelerometer output, upper word
#define Z_ACCL_LOW  	0x18  //Z-axis accelerometer output, lower word
#define Z_ACCL_OUT  	0x1A  //Z-axis accelerometer output, upper word
#define TEMP_OUT    	0x1C  //Temperature output (internal, not calibrated)
#define TIME_STAMP  	0x1E  //PPS mode time stamp
#define X_DELTANG_LOW	0x24  //X-axis delta angle output, lower word
#define X_DELTANG_OUT	0x26  //X-axis delta angle output, upper word
#define Y_DELTANG_LOW	0x28  //Y-axis delta angle output, lower word
#define Y_DELTANG_OUT	0x2A  //Y-axis delta angle output, upper word
#define Z_DELTANG_LOW	0x2C  //Z-axis delta angle output, lower word
#define Z_DELTANG_OUT	0x2E  //Z-axis delta angle output, upper word
#define X_DELTVEL_LOW	0x30  //X-axis delta velocity output, lower word
#define X_DELTVEL_OUT	0x32  //X-axis delta velocity output, upper word
#define Y_DELTVEL_LOW	0x34  //Y-axis delta velocity output, lower word
#define Y_DELTVEL_OUT	0x36  //Y-axis delta velocity output, upper word
#define Z_DELTVEL_LOW	0x38  //Z-axis delta velocity output, lower word
#define Z_DELTVEL_OUT	0x3A  //Z-axis delta velocity output, upper word
#define XG_BIAS_LOW		0x40  //X-axis gyroscope bias offset correction, lower word
#define XG_BIAS_HIGH	0x42  //X-axis gyroscope bias offset correction, upper word
#define YG_BIAS_LOW		0x44  //Y-axis gyroscope bias offset correction, lower word
#define YG_BIAS_HIGH	0x46  //Y-axis gyroscope bias offset correction, upper word
#define ZG_BIAS_LOW		0x48  //Z-axis gyroscope bias offset correction, lower word
#define ZG_BIAS_HIGH	0x4A  //Z-axis gyroscope bias offset correction, upper word
#define XA_BIAS_LOW		0x4C  //X-axis accelerometer bias offset correction, lower word
#define XA_BIAS_HIGH	0x4E  //X-axis accelerometer bias offset correction, upper word
#define YA_BIAS_LOW		0x50  //Y-axis accelerometer bias offset correction, lower word
#define YA_BIAS_HIGH	0x52  //Y-axis accelerometer bias offset correction, upper word
#define ZA_BIAS_LOW		0x54  //Z-axis accelerometer bias offset correction, lower word
#define ZA_BIAS_HIGH	0x56  //Z-axis accelerometer bias offset correction, upper word
#define FILT_CTRL    	0x5C  //Filter control
#define MSC_CTRL    	0x60  //Miscellaneous control
#define UP_SCALE    	0x62  //Clock scale factor, PPS mode
#define DEC_RATE    	0x64  //Decimation rate control (output data rate)
#define NULL_CFG    	0x66  //Auto-null configuration control
#define GLOB_CMD    	0x68  //Global commands
#define FIRM_REV    	0x6C  //Firmware revision
#define FIRM_DM    		0x6E  //Firmware revision date, month and day
#define FIRM_Y    		0x70  //Firmware revision date, year
#define PROD_ID    		0x72  //Product identification 
#define SERIAL_NUM    0x74  //Serial number (relative to assembly lot)
#define USER_SCR1    	0x76  //User scratch register 1 
#define USER_SCR2    	0x78  //User scratch register 2 
#define USER_SCR3    	0x7A  //User scratch register 3 
#define FLSHCNT_LOW   0x7C  //Flash update count, lower word 
#define FLSHCNT_HIGH  0x7E  //Flash update count, upper word 


/**
 * 用来存储Burst Read方式下读到的字节流
 * 启动的时候将CS线拉低，随后发送一个字0x6800，然后连续读取SPI数据
 */
typedef struct 
{
	int16_t diag_stat;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
	int16_t x_accl;
	int16_t y_accl;
	int16_t z_accl;
	int16_t temp;
	int16_t data_cntr;
	int16_t checknum;
} ADX_t;
typedef struct
{
	float wx;//角速度
	float wy;
	float wz;
	float accx;//加速度
	float accy;
	float accz;
	float anglex;//角度
	float angley;
	float anglez;
}GyroData_t;
extern ADX_t imu;
extern GyroData_t GyroData;
int8_t ADX_BurstRead(void);
int8_t ADX_Read_Reg(uint8_t* addr_Reg,uint16_t* Rx_point,uint8_t Reg_num);
int8_t ADX_Write_Reg(uint8_t addr,uint8_t value);
void ADX_Init(void);//为保证spi数据的burst read 不在进行中 先大量读取
void ADX_Single_Handle(void);//采集一次数据
void Self_Calibration(void);
#endif
