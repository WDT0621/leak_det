#include "mpu9250.h"
#include "myiic.h"
//#include "bsp_SysTick.h"
//#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F746������
//MPU9250��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/12/30
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��MPU9250
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t ICM20948_Init(void)
{
    uint8_t res=0;
    IIC_Init();     //��ʼ��IIC����
    res = MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_1,0X80);//��λMPU9250
    HAL_Delay(100);  //��ʱ100ms
    res = MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_1,0X00);//����MPU9250
    res = MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
	  res = MPU_Set_Accel_Fsr(0);					       	 	//���ٶȴ�����,��2g
    res = MPU_Set_Rate(50);						       	 	//���ò�����50Hz
//    MPU_Write_Byte(ICM20948_ADDR,MPU_INT_EN_REG,0X00);   //�ر������жϣ��ϵ�Ĭ��0x00
//	  MPU_Write_Byte(ICM20948_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
//	  MPU_Write_Byte(ICM20948_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO���������ã��ϵ�Ĭ��0x00
	  res = MPU_Write_Byte(ICM20948_ADDR,ICM20948_INT_PIN_CFG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������

 		    res=MPU_Read_Byte(ICM20948_ADDR,ICM20948_WHO_AM_I);  //��ȡMPU6500��ID
    if(res==ICM20948_ID) //����ID��ȷ
    {
        MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_1,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(ICM20948_ADDR,ICM20948_PWR_MGMT_2,0X00);  	//���ٶ��������Ƕ�����
		    MPU_Set_Rate(50);						       	//���ò�����Ϊ50Hz   
    }else return 1;
		
    res=MPU_Read_Byte(AK09918_ADDR,AK09916__WIA2);    			//��ȡAK8963 ID   
    if(res==AK09918_ID)
    {
        MPU_Write_Byte(AK09918_ADDR,AK09916__CNTL,0X01);		//����AK8963Ϊ���β���ģʽ
    }else return 1;
		

    return 0;
}

//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(ICM20948_ADDR,ICM20948_GYRO_CONFIG_1,fsr<<1);//���������������̷�Χ����������һλ��Ϊ�������̷�Χ��λ
}
//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(ICM20948_ADDR,ICM20948_ACCEL_CONFIG,fsr<<1);//���ü��ٶȴ����������̷�Χ  
}

//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(ICM20948_ADDR,ICM20948_GYRO_CONFIG_1,data<<4);//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(ICM20948_ADDR,ICM20948_GYRO_SMPLRT_DIV,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
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
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(AK09918_ADDR,AK09916_XOUT_L,6,buf);//IIC��ȡ��������ֵ������ִ�д�����
	HAL_Delay(10);
	if(res==0)
	{
		*mx=((uint16_t)buf[1]<<8)|buf[0];  
		*my=((uint16_t)buf[3]<<8)|buf[2];  
		*mz=((uint16_t)buf[5]<<8)|buf[4];
	} 	
	  MPU_Write_Byte(AK09918_ADDR,AK09916__CNTL,0X01);//ÿ��ִ���궼Ҫ���õ��β���ģʽ
    return res;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //��������
        if(IIC_Wait_Ack())      //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
//			IIC_Wait_Ack();
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(data);        //��������
    if(IIC_Wait_Ack())          //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    res=IIC_Read_Byte(0);		//������,����nACK  
    IIC_Stop();                 //����һ��ֹͣ����
    return res;  
}
