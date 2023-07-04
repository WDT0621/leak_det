/** 
* ADIS16470简易功能(基于STM32的HAL库)
*	在Cube中启用任意一个SPI设备为全双工,注意参数CPOL为HIGH,CHPA=2,如果需要使用BurstRead还要使波特率小于1Mb/s,否则使波特率小于2Mb/s
*	除了启用一个SPI外还需要启用一个GPIO_Output为SPI设备片选线,方式为开漏(上拉)输出
*	将check声明为unsigned char其他同上
*/  
#include "ADXIS.h"
#include "spi.h"
#include "string.h"
#include "stdint.h"

#define ADX_SPI hspi1 		//使用哪个spi

ADX_t imu;								//读到的数据全局变量
GyroData_t GyroData;
/** 
* 该库使用默认模式,陀螺仪原始数据更新频率2000Hz
*/

static int16_t Send_Cmd=0x6800;//Burst Read指令
static void sb_delay(volatile uint32_t t)
{
	while(t--);
}
//单个数据的传送
void ADX_Init(void)//为保证spi数据的burst read 不在进行中 先大量读取
{
	ADX_Write_Reg(GLOB_CMD,0x80);	//重启指令		SOFTWARE RESET COMMAND：GLOB_CMD[7] = 1
	ADX_Write_Reg(0x69,0x00);
	HAL_Delay(400);						//等待重启完成
	
//	ADX_Write_Reg(FILT_CTRL,0x03);	//设置内置巴特沃斯滤波器等级
//	ADX_Write_Reg(0x5D,0x00);				// 手册P32说这两个寄存器的值可以影响数据的位宽，没懂

//	ADX_Write_Reg(DEC_RATE,0x03);		//设定均值滤波器值为3 此时角度差最终输出频率为2000/(3+1)=500Hz
//	ADX_Write_Reg(0x65,0x00);	// 2000/(DEC_RATE + 1)
}
static uint16_t ADX_flame_TandR(uint16_t trans)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	uint16_t result;
	static HAL_StatusTypeDef state;
	state=HAL_SPI_TransmitReceive(&ADX_SPI,(uint8_t*)&trans,(uint8_t*)&result,1,0xff);
	if(state!=HAL_OK) {
		while(1);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	sb_delay(150);
	return result;
}
//读取寄存器的函数,因为SPI的方式 在连续读取上有效率优势,所以建议连续读取
/*
@parameter:
	addr_Reg 待读取的寄存器数组,Rx_point
	Rx_point 读取后的数据放的位置
	Reg_num	 欲读取的寄存器数量
@return:
	0 成功
	1 错误  //emmmm现在暂时还没有实现错误码
*/
int8_t ADX_Read_Reg(uint8_t* addr_Reg,uint16_t* Rx_point,uint8_t Reg_num)
{
	uint16_t Tx_tmp=0,Rx_tmp=0;
	
	//数据第一帧 只发不接
	Tx_tmp=addr_Reg[0]<<8;
	Rx_tmp=ADX_flame_TandR(Tx_tmp);
	for(uint8_t i=1;i<Reg_num;i++)//+1是因为spi有一帧延迟
	{
		Tx_tmp=addr_Reg[i]<<8;//准备发送帧的格式
		Rx_tmp=ADX_flame_TandR(Tx_tmp);
		Rx_point[i-1]=Rx_tmp;
	}
	//数据最后一帧 只接不发
	Tx_tmp=0;
	Rx_point[Reg_num-1]=ADX_flame_TandR(Tx_tmp);
	
	return 0;
}
/** 
* 对ADIS16470寄存器内部写操作函数
* addr 写的地址
* value 写的值
*/
int8_t ADX_Write_Reg(uint8_t addr,uint8_t value)
{
	addr|=0x80;//写数据的掩码
	uint16_t Tx_tmp=(addr<<8) | value;
	ADX_flame_TandR(Tx_tmp);
	return 0;
}
/** 
* 采用BurstRead一次性获得三轴加速度与角速度,16位精度 
* 这种方式限制时钟1M以内
*	这里有些全双工的意思，发送要读取的寄存器地址后，下一个时钟结构就返回，
* 可以连续发送寄存器地址，接受的时候加上偏移量
* 读取完要按小端，将两个16位合成32位，32位数还要看手册，手册表12有换算关系
*/
int8_t ADX_BurstRead()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	static uint8_t* u8point=(uint8_t*)&imu;
	int16_t parity=0;
	HAL_StatusTypeDef check=HAL_OK;
	uint16_t tmpRx=0;
	static uint16_t tmpTx[10]={0};
	check|=HAL_SPI_TransmitReceive(&ADX_SPI,(uint8_t*)&Send_Cmd,(uint8_t*)&tmpRx,1,0xff);//发送指令,该16位接收无意义,故不保存数据
	//注意到数据手册中ADXIS16470的发送格式是低地址在前 高地址在后 符合stm32的小端模式 所以不需要做移位处理
	if(check!=HAL_OK)while(1);
	sb_delay(100);
	check|=HAL_SPI_TransmitReceive(&ADX_SPI,(uint8_t*)&tmpTx,u8point,10,0xff);//接收20字节的数据放入imu结构体中
	if(check!=HAL_OK)while(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	sb_delay(150);
	//进行数据校验
	for(uint8_t i=0;i<9*2;i++)
	{
		parity+=u8point[i];
	}
	if(parity==imu.checknum && !check)return 0;
	else 
	{
		memset(&imu,0x00,sizeof(imu));
		return -1;//校验失败
	}
}
/** 
* 采用读寄存器的方式获得陀螺仪三轴加速度与三轴角速度与三轴姿态角,32位精度
* 这种情况时钟要求2M以内
* 更高精度
*/
void ADX_Single_Handle(void)
{
	//用于保存上一次的角度值 等下加回去
	float lastx=GyroData.anglex;
	float lasty=GyroData.angley;
	float lastz=GyroData.anglez;
	
//	static uint8_t addr[]=
//	{ X_GYRO_OUT,X_GYRO_LOW,//Gyro_X
//		X_GYRO_OUT,Y_GYRO_LOW,//Gyro_Y
//		Z_GYRO_OUT,Z_GYRO_LOW,//Gyro_Z
//		X_ACCL_OUT,X_ACCL_LOW,//Acc_X
//		Y_ACCL_OUT,Y_ACCL_LOW,//Acc_Y
//		Z_ACCL_OUT,Z_ACCL_LOW,//Acc_Z
//		X_DELTANG_OUT,X_DELTANG_LOW,//Delta_X 注意这里读到的是2ms内的角度差
//		Y_DELTANG_OUT,Y_DELTANG_LOW,//Delta_Y
//		Z_DELTANG_OUT,Z_DELTANG_LOW,//Delta_Z
//	};
	
	static uint8_t addr[]=
	{ X_GYRO_LOW,X_GYRO_OUT,//Gyro_X
		Y_GYRO_LOW,Y_GYRO_OUT,//Gyro_Y
		Z_GYRO_LOW,Z_GYRO_OUT,//Gyro_Z
		X_ACCL_LOW,X_ACCL_OUT,//Acc_X
		Y_ACCL_LOW,Y_ACCL_OUT,//Acc_Y
		Z_ACCL_LOW,Z_ACCL_OUT,//Acc_Z
		X_DELTANG_LOW,X_DELTANG_OUT,//Delta_X 注意这里读到的是2ms内的角度差
		Y_DELTANG_LOW,Y_DELTANG_OUT,//Delta_Y
		Z_DELTANG_LOW,Z_DELTANG_OUT,//Delta_Z
	};
	
	
	static uint16_t data[sizeof(addr)];
	static int32_t temp[9] = {0};
	int32_t* point32=(int32_t*)data;//便于将两个int16_t类型合成int32_t类型
	float* Gyro_float=(float*)&GyroData;//便于将int32_t类型转换成float类型
	
	
	
	
	ADX_Read_Reg(addr,data,sizeof(addr));
	//52428.8f
	uint8_t j;
	for (j = 0; j < 9; ++j) {
		temp[j] = *(point32 + j);
	}
	
	
	uint8_t i;
	for(i=0;i<3;i++)//Gyro 0.1 °/s=2^16LSB
	{
		// *(Gyro_float+i)=*(point32+i)/655360.0f;
		*(Gyro_float+i) = *(point32+i) * 0.00000152587890625;
	}
	for(;i<6;i++)//Acc 1.25 m/s^2=2^16LSB
	{
		// *(Gyro_float+i)=*(point32+i)/52428.8f;
		*(Gyro_float+i) = *(point32+i) * 0.000019073486328125;
	}
	for(;i<9;i++)//Angle 2160 °=2^31LSB
	{
		// *(Gyro_float+i)=*(point32+i)/994205.4f;
		*(Gyro_float+i) = *(point32+i) * 0.000001005828380584716796875;
	}
	//加回角度值 
	GyroData.anglex+=lastx;
	GyroData.angley+=lasty;
	GyroData.anglez+=lastz;
}
/** 
* 对陀螺仪进行零偏校准
* 该函数建议在开始时执行,执行时陀螺仪最好不要移动 因为陀螺仪可以保存校准值,所以校准不一定是每次启动时必须的
*/
void Self_Calibration(void)
{
	int32_t RawBiasData[6]={0};		//存储二进制的当前误差值
	uint8_t addr = XG_BIAS_LOW;		//XG_BIAS_LOW的地址因为其后面的地址是连续的所以取这个开始
	uint8_t* writedata=(uint8_t*)RawBiasData;//待写入内存的首地址 和addr一起递增
	for(uint8_t i=0;i<24;i++)			//24是6个校准值*4 因为一个校准值是4个字节的 现在先执行一次进行清零
	{
		ADX_Write_Reg(addr+i,writedata[i]);
	}
	uint16_t count=0;
	uint32_t timestamp;
	GyroData_t GyroInt={0};//用于积分用的临时结构体
	while(count<5000)//校准10秒
	{
		ADX_Single_Handle();
		timestamp=HAL_GetTick();
		while(HAL_GetTick()-timestamp<2);
		count++;
	}
	GyroInt.anglex=GyroData.anglex;
	GyroInt.angley=GyroData.angley;
	GyroInt.anglez=GyroData.anglez;
	//执行到这里说明数据采集完了
	float* pointf=(float*)&GyroInt;//浮点地址 便于从Gyro_Data中获取数据
	uint8_t i=0;
	for(;i<3;i++)
	{
		RawBiasData[i]=-(*(pointf+i+6)) *131072.0f/2.0f;//将他们重新转换成int32_t 注意这时候校准值取的是角度值 5s的角度变化/5就等于角速度了 所以此处除以5
	}
	//执行到这里 所有校准数据准备完成 准备写入
	for(uint8_t i=0;i<12;i++)//12是3个校准值*4 因为一个校准值是4个字节的
	{
		ADX_Write_Reg(addr+i,writedata[i]);
	}
}
