/**
 ******************************************************************************
 * @file    lis3mdl.c
 * @author  MCD Application Team + WDT
 * @brief   This file provides a set of functions needed to manage the LIS3MDL
 *          magnetometer devices
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "lis3mdl.h"

/**
  * @}
  */ 
uint8_t lis3[6];								//读到的数据全局变量	

/**	自己多封装一层，给片上WriteAddr地址寄存器写入pBuffer指向的一Byte内容
	*
	*/
uint32_t I2C_LIS3_ByteWrite(uint8_t* pBuffer, uint8_t WriteAddr) {
		HAL_StatusTypeDef status = HAL_OK;
		status = HAL_I2C_Mem_Write(&hi2c2, LIS3MDL_MAG_I2C_ADDRESS_LOW, (uint16_t)
		WriteAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 1, 100);
		/* Check the communication status */
		if (status != HAL_OK) {
		/* Execute user timeout callback */
				return status;
		}
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
		}
		return status;
}
/**	自己多封装一层，给片上WriteAddr地址寄存器写入pBuffer指向的NumByteToWrite个Byte
	*
	*/
uint32_t I2C_LIS3_BufferWrite(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite) {
		HAL_StatusTypeDef status = HAL_OK;
		/* Write EEPROM_PAGESIZE */
		status=HAL_I2C_Mem_Write(&hi2c2, LIS3MDL_MAG_I2C_ADDRESS_LOW, WriteAddr, 
		I2C_MEMADD_SIZE_8BIT, (uint8_t*)(pBuffer),NumByteToWrite, 100);
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
		}
		/* Wait for the end of the transfer */
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
		}
		return status;
}
/**	自己多封装一层，从片上ReadAddr地址寄存器读入NumByteToWrite个Byte到pBuffer指向的内存
	*
	*/
uint32_t I2C_LIS3_BufferRead(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead) {
		HAL_StatusTypeDef status = HAL_OK;
		status=HAL_I2C_Mem_Read(&hi2c2, LIS3MDL_MAG_I2C_ADDRESS_LOW, ReadAddr, 
		I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuffer, NumByteToRead, 1000);
		return status;
}
/**	LIS3MDL初始化函数，检查器件ID
	*	这个地方可以需要了再采，频率1Hz都不到，看后面是否需要
	*	-1:IIC读写错误，	-2:器件ID错误
	*/
int LIS3MDL_MagInit() {  
		uint8_t rdbuf[1];
		uint8_t wrbuf[1];
		if (I2C_LIS3_BufferRead(rdbuf, LIS3MDL_MAG_WHO_AM_I_REG, 1)) {
				return -1;
		}
		if (rdbuf[0] != I_AM_LIS3MDL) {
				return -2;
		}
		/* 设置五个控制寄存器 */
		wrbuf[0] = LIS3MDL_MAG_SOFT_RESET_ENABLE;			// RGE2:软件reset，量程默认4Gaus就不设了
		if (I2C_LIS3_ByteWrite(wrbuf, LIS3MDL_MAG_CTRL_REG2)) {
				return -1;
		}
		HAL_Delay(1);
		
		wrbuf[0] = LIS3MDL_MAG_OM_XY_ULTRAHIGH | 0x82;	// REG1:XY轴高性能模式，155Hz，Fast_ODR=1启动,启动温度传感器
		if (I2C_LIS3_ByteWrite(wrbuf, LIS3MDL_MAG_CTRL_REG1)) {
				return -1;
		}
		wrbuf[0] = LIS3MDL_MAG_CONTINUOUS_MODE;				// REG3:连续采样模式
		if (I2C_LIS3_ByteWrite(wrbuf, LIS3MDL_MAG_CTRL_REG3)) {
				return -1;
		}
		wrbuf[0] = LIS3MDL_MAG_OM_Z_ULTRAHIGH;					// REG4:Z轴高性能模式
		if (I2C_LIS3_ByteWrite(wrbuf, LIS3MDL_MAG_CTRL_REG4)) {
				return -1;
		}
		return 0;
}


///**
//  * @brief  Set/Unset Magnetometer in low power mode.
//  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
//  */
//void LIS3MDL_MagLowPower(uint16_t status)
//{  
//  uint8_t ctrl = 0;
//  
//  /* Read control register 1 value */
//  ctrl = SENSOR_IO_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3);

//  /* Clear Low Power Mode bit */
//  ctrl &= ~(0x20);

//  /* Set Low Power Mode */
//  if(status)
//  {
//    ctrl |= LIS3MDL_MAG_CONFIG_LOWPOWER_MODE;
//  }else
//  {
//    ctrl |= LIS3MDL_MAG_CONFIG_NORMAL_MODE;
//  }
//  
//  /* write back control register */
//  SENSOR_IO_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, ctrl);  
//}

/**
  * @brief  Read X, Y & Z Magnetometer values 
  * @param  pData: Data out pointer
  */
int LIS3MDL_MagReadXYZ_int16(int16_t* pData)
{
		int16_t pnRawData[3];
		uint8_t ctrlm[1] = {0};
		uint8_t buffer[6];
		uint8_t i = 0;
		float sensitivity = 0;
		
		/* Read the magnetometer control register content */
		if (I2C_LIS3_BufferRead(ctrlm, LIS3MDL_MAG_CTRL_REG2, 1)) {
				return -1;
		}
		/* Read output register X, Y & Z acceleration */
		if (I2C_LIS3_BufferRead(buffer, LIS3MDL_MAG_OUTX_L, 6)) {
				return -1;
		}
		for(i=0; i<3; i++) {
				// pnRawData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
				pData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
		}
		
//		/* Normal mode */
//		/* Switch the sensitivity value set in the CRTL_REG2 */
//		switch(ctrlm[0] & 0x60) {
//		case LIS3MDL_MAG_FS_4_GA:
//				sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA;
//				break;
//		case LIS3MDL_MAG_FS_8_GA:
//				sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA;
//				break;
//		case LIS3MDL_MAG_FS_12_GA:
//				sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA;
//				break;
//		case LIS3MDL_MAG_FS_16_GA:
//				sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA;
//				break;    
//		}
//		
//		/* Obtain the mGauss value for the three axis */
//		for(i=0; i<3; i++) {
//				pData[i]=( int16_t )(pnRawData[i] * sensitivity);
//		}
		return 0;
}

/* 减轻负担版本 */
int LIS3MDL_MagReadXYZ_uint8(uint8_t* pData) {
		/* Read output register X, Y & Z acceleration */
		if (I2C_LIS3_BufferRead(pData, LIS3MDL_MAG_OUTX_L, 6)) {
				return -1;
		}
		return 0;
}




int LIS3MDL_MagReadTMP(int16_t* pData)
{
		uint8_t buffer[2];
		/* Read output register X, Y & Z acceleration */
		if (I2C_LIS3_BufferRead(buffer, LIS3MDL_MAG_TEMP_OUT_L, 2)) {
				return -1;
		}
		*pData = ((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);
		return 0;
}


//static void LIS2MDL_Write_Offset(CPU_INT16S x_offset,CPU_INT16S y_offset,CPU_INT16S z_offset)
//{
//    CPU_INT08U buf[LIS2MDL_OFFSETBUF_LEN];
// 
//    buf[0] =  x_offset & 0xFF;
//    buf[1] = (x_offset >> 8) & 0xFF;
//    buf[2] =  y_offset & 0xFF;
//    buf[3] = (y_offset >> 8) & 0xFF;
//    buf[4] =  z_offset & 0xFF;
//    buf[5] = (z_offset >> 8) & 0xFF;
// 
//    LIS2MDL_HardI2C_Register_Write(LIS2MDL_ADDR_WRITE, LIS2MDL_ADDR_OFFSETX_L, buf, LIS2MDL_OFFSETBUF_LEN);
//}
/**
  * @}
  */ 

/**
  * @}
  */
  
/**
  * @}
  */
  
/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
  
