#include "mpu9250.h"
#include "myiic.h"
//#include "bsp_SysTick.h"
//#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F746开发板
//MPU9250驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/12/30
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化MPU9250
//返回值:0,成功
//    其他,错误代码
uint8_t ICM20948_Init(void)
{
    uint8_t res=0;
    IIC_Init();     //初始化IIC总线
    res = MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_1,0X80);//复位MPU9250
    HAL_Delay(100);  //延时100ms
    res = MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_1,0X00);//唤醒MPU9250
    res = MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	  res = MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
    res = MPU_Set_Rate(50);						       	 	//设置采样率50Hz
//    MPU_Write_Byte(ICM20948_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断，上电默认0x00
//	  MPU_Write_Byte(ICM20948_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
//	  MPU_Write_Byte(ICM20948_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO，不用设置，上电默认0x00
	  res = MPU_Write_Byte(ICM20948_ADDR,ICM20948_INT_PIN_CFG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计

 		    res=MPU_Read_Byte(ICM20948_ADDR,ICM20948_WHO_AM_I);  //读取MPU6500的ID
    if(res==ICM20948_ID) //器件ID正确
    {
        MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_1,0X01);  	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_2,0X00);  	//加速度与陀螺仪都工作
		    MPU_Set_Rate(50);						       	//设置采样率为50Hz   
    }else return 1;
		
    res=MPU_Read_Byte(AK09918_ADDR,AK09916__WIA2);    			//读取AK8963 ID   
    if(res==AK09918_ID)
    {
        MPU_Write_Byte(AK09918_ADDR,AK09916__CNTL,0X01);		//设置AK8963为单次测量模式
    }else return 1;
		

    return 0;
}

//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(ICM20948_ADDR,ICM20948_GYRO_CONFIG_1,fsr<<1);//设置陀螺仪满量程范围，这里左移一位即为设置量程范围的位
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(ICM20948_ADDR,ICM20948_ACCEL_CONFIG,fsr<<1);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(ICM20948_ADDR,ICM20948_GYRO_CONFIG_1,data<<4);//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(ICM20948_ADDR,ICM20948_GYRO_SMPLRT_DIV,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
//short MPU_Get_Temperature(void)
//{
//    u8 buf[2]; 
//    short raw;
//	float temp;
//	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
//    raw=((u16)buf[0]<<8)|buf[1];  
//    temp=21+((double)raw)/333.87;  
//    return temp*100;;
//}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res; 
	res=MPU_Read_Len(ICM20948_ADDR,ICM20948_GYRO_XOUT_H,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(ICM20948_ADDR,ICM20948_ACCEL_XOUT_H,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//得到磁力计值(原始值)
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(AK09918_ADDR,AK09916_XOUT_L,6,buf);//IIC读取磁力计数值，这里执行错误了
	HAL_Delay(10);
	if(res==0)
	{
		*mx=((uint16_t)buf[1]<<8)|buf[0];  
		*my=((uint16_t)buf[3]<<8)|buf[2];  
		*mz=((uint16_t)buf[5]<<8)|buf[4];
	} 	
	  MPU_Write_Byte(AK09918_ADDR,AK09916__CNTL,0X01);//每次执行完都要设置单次采样模式
    return res;
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //发送数据
        if(IIC_Wait_Ack())      //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
//			IIC_Wait_Ack();
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(data);        //发送数据
    if(IIC_Wait_Ack())          //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    res=IIC_Read_Byte(0);		//读数据,发送nACK  
    IIC_Stop();                 //产生一个停止条件
    return res;  
}
