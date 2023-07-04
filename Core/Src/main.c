/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
	
/**				ģ������ADC���ø�
 *	MIC1_OUT-------PA1(24)
 *	MIC2_OUT-------PA2(25)
 *				�¹ߵ�Ҫ��
 *	SPI1_NSS-------PA4(29)
 *	SPI1_SCK-------PA5(30)
 *	SPI1_MISO------PA6(31)
 *	SPI1_MOSI------PA7(32)
 *	DR 			-------PC4(33)
 *	SYNC		-------PC5(34)
 *				USB�ӿڲ��ø�
 *	USB_D+  -------PA11(70)
 *	USB_D-  -------PA12(71)
 *				SD�����ø�
 *	SD_DATA0-------PC8(65)
 *	SD_DATA1-------PC9(66)
 *	SD_DATA2-------PC10(78)
 *	SD_DATA3-------PC11(79)
 *	SD_CLK---------PC12(80)
 *	SD_CMD---------PD2(83)
 *	SD_DETECT------PD3(84)
 *				�´�����Ҫ��
 *	LIS3M_INT------PE14(45)
 *	LIS3M_RD-------PE15(46)
 *	LIS3M_SCL------PB10(47)
 *	LIS3M_SDA------PB11(48)
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"

#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADXIS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */

FIL filobj;			// ��Ҫ���������ļ�
UINT brs;
FIL fADI;				// �ߵ�������ADIS16470�ļ�����
FIL fICM;				// �ߵ�������ICM20948�ļ�����
FIL fLM;				// ģ�������������ļ�����

extern volatile uint8_t bDeviceState;		//USB���� ���
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t flag_1=0;			//�жϲɼ����ݾ;���,��д�����������ļ���־,�տ�ʼû��д
uint8_t flag_2=0;			//�жϲɼ����ݾ;���,��д�ߵ��������ļ���־,�տ�ʼû��д

uint8_t BZ_1=1;				//д�����ļ���������ɣ��ɽ��������ɼ�,�տ�ʼ���Ȳɼ�
uint8_t BZ_2=1;				//д�ߵ��ļ���������ɣ��ɽ��йߵ��ɼ�,�տ�ʼ���Ȳɼ�

#define DMA_Size 750  //��������
uint16_t ADC_DMA_ConvertedValue_0[DMA_Size*2];//DMA˫����
uint16_t ADC_DMA_ConvertedValue_1[DMA_Size*2];
extern	uint8_t adcx[DMA_Size*3];
extern	uint8_t ttbuf[180];
FRESULT fr;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
		/* USER CODE BEGIN 1 */
							
		//-------------------------------------------------------------		
		static int res;
		int lsk_1 = 100;	
		int lsk_2 = 100;
		uint8_t information[] = "�������:smartball_1\n��ǰ�ļ�:";//32	
		uint8_t inf_file_ctent[42];										//��information�ļ�����
		int inf_file_size = 0;												//information�ļ���ʵ�����ݴ�С
		int index_bit = 0;														//��¼���λ��
		uint8_t biao[10];															//����һ���ܴ������Ϊ�������
		int sum = 0;
		int m;
		int n;																				//Ȩ��
		char dir_path[40] = "0:data_";								//�ļ�·��
		int len;
		// uint8_t ICM[12] 	= "ICM20948.dat";
		uint8_t ADIS[13] 	= "ADIS16470.dat";
		uint8_t SHP[11] 	= "SHP0645.dat";
		uint8_t LM[9] 		= "LM386.dat";
		char fil_path_1[40];
		// char fil_path_2[40];
		char fil_path_3[40];
		uint8_t ICM20948[100] = "����������:�ߵ�������ICM20948\n����ʱ��:2022��03��01��10:00\n�������:100ms\n";
		uint8_t ADIS16470[100]= "����������:�ߵ�������ADIS16470\n����ʱ��:2022��03��01��10:00\n�������:100ms\n";
		uint8_t SHP0645[100] 	= "����������:��Ƶ������SHP0645\n����ʱ��:2022��03��01��10:00\n�������:100ms\n";
		uint8_t LM386[100] 		= "����������:��Ƶ������LM386\n����ʱ��:2022��03��01��10:00\n�������:100ms\n";
		
		
		static HAL_StatusTypeDef status;
		/* USER CODE END 1 */

		/* MCU Configuration--------------------------------------------------------*/

		/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
		HAL_Init();

		/* USER CODE BEGIN Init */

		/* USER CODE END Init */

		/* Configure the system clock */
		SystemClock_Config();

		/* USER CODE BEGIN SysInit */
		// MX_TIM2_Init();
		HAL_Delay(50);
		/* USER CODE END SysInit */

		/* Initialize all configured peripherals */
		MX_GPIO_Init();
		MX_DMA_Init();
		MX_SDIO_SD_Init();
		MX_FATFS_Init();
		MX_USB_DEVICE_Init();
		MX_ADC1_Init();
		MX_TIM2_Init();
		// MX_TIM3_Init();
		// MX_TIM4_Init();
		MX_SPI1_Init();
		MX_I2C2_Init();
		/* USER CODE BEGIN 2 */
		HAL_Delay(50);
		ADX_Init();
		HAL_Delay(50);
		//res = LIS3MDL_MagInit();
		
	//	while (1) {
	//	}
		
		/*##-2- Register the file system object to the FatFs module ##############*/
			if(f_mount(&SDFatFs, /*(TCHAR const*)SDPath*/"0:", 0) != FR_OK) {
				/* FatFs Initialization Error */
				Error_Handler();
			}
			//�½�information�ļ�������š�д�±�š�д�ļ�·��
			f_mkdir("0:information_file");
			//��information��д��ǰ����Ϣ
			fr = f_open(&filobj,"0:information_file/information.inf",FA_CREATE_NEW|FA_WRITE);
			fr = f_write(&filobj,information,32,(void *)&brs);
			f_close(&filobj);
			
			fr = f_open(&filobj,"0:information_file/information.inf",FA_WRITE|FA_READ);
			fr = f_read(&filobj,inf_file_ctent,42,&brs);
			inf_file_size=f_size(&filobj);//���ļ�����ʵ�ʳ���
			index_bit=inf_file_size-32;
				
			for(int i = 0; i < index_bit; i++) {//ȡ����ŵõ�������
				biao[i] = inf_file_ctent[32+i];
				sum *= 10;
				biao[i] = biao[i] - 0x30;
				sum = sum + biao[i];
			}
			
			if((sum==9)|(sum==99)|(sum==999)|(sum==9999)|(sum==99999)|(sum==999999)|(sum==9999999)){
				index_bit = index_bit + 1;		//�ж��Ƿ���λ���仯
			}
			sum = sum + 1;
			
			for(int i = 0; i < index_bit; i++) {//��ԭΪASCII������
				m = index_bit - i - 1;
				n = 1;
				for(int j = 0; j < m; j++) {
					n = n * 10;
				}
				biao[i] = (sum / n) + 0x30;
				sum = sum % n;
			}
			
			//�Ѹ��º�ı�Ŵ���information�ļ���Ϊ�´��ϵ��½��ļ���׼��
			f_close(&filobj);
			fr = f_open(&filobj,"0:information_file/information.inf",FA_READ|FA_WRITE);
			f_lseek(&filobj,32);//ȷ��һ��ָ�뵽��ָ����
			f_write(&filobj,biao,index_bit,&brs);//���º�ı�����ݺ�λ��
			f_close(&filobj);
			
			
			len=strlen(dir_path);//ȡ�ַ�������ʵ�ʳ���
			for(int i = 0; i < index_bit; i++) {
				dir_path[len+i]=biao[i];//�����ַ�
			}
			f_mkdir((const TCHAR*)&dir_path);//�½������ļ���
			for(int i = 0; i < (7+index_bit); i++)
			{
				fil_path_1[i]=dir_path[i];
	//			fil_path_2[i]=dir_path[i];
				fil_path_3[i]=dir_path[i];
			}
			len=strlen(dir_path);
			fil_path_1[len]=47;
	//		fil_path_2[len]=47;
			fil_path_3[len]=47;
			for(int i = 0; i < 13; i++)
			fil_path_1[len+1+i] = ADIS[i];
	//		for(int i = 0; i < 11; i++)
	//		fil_path_2[len+1+i] = SHP[i];
			for(int i = 0; i < 9; i++)
			fil_path_3[len+1+i] = LM[i];
			
			//�½������ļ�
			fr = f_open(&fADI, (const TCHAR*)&fil_path_1, FA_CREATE_NEW | FA_WRITE);
			fr = f_write(&fADI,ADIS16470,100,&brs);
			f_sync(&fADI);
			
			fr = f_open(&fLM, (const TCHAR*)&fil_path_3, FA_CREATE_NEW | FA_WRITE);	
			fr = f_write(&fLM, LM386, 100, &brs);
			f_sync(&fLM);
			

		
			MX_TIM3_Init();//50Hz������30SҪд��27KB
			MX_TIM4_Init();//˫ͨ����������30Sд��2250KB
			
			HAL_DMAEx_MultiBufferStart_IT((&hadc1)->DMA_Handle,(uint32_t)&hadc1.Instance->DR,(uint32_t)&ADC_DMA_ConvertedValue_0,(uint32_t)&ADC_DMA_ConvertedValue_1,DMA_Size*2);
			status = HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_DMA_ConvertedValue_0, DMA_Size*2);
		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
		while (1)
		{


			if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)//USBû���ӿ�д����
			{
				if(flag_1==1)//д����������
				{
					fr=f_lseek(&fLM,lsk_1);
					lsk_1=lsk_1+(DMA_Size*3);
					fr=f_write(&fLM,adcx,DMA_Size*3,&brs);
					f_sync(&fLM);
					flag_1=0;
					BZ_1=1;
				}
				if(flag_2==1)//д�ߵ�����
				{
					fr=f_lseek(&fADI,lsk_2);
					lsk_2=lsk_2+180;
					fr=f_write(&fADI,ttbuf,180,&brs);
					f_sync(&fADI);
					flag_2=0;
					BZ_2=1;
				}
			}
			
			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */
		}
		/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_Delay_us(uint16_t differ)
{										     	    
    __HAL_TIM_SetCounter(&htim2,0);	
    while(__HAL_TIM_GetCounter(&htim2)<differ)
    {
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
