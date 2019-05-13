/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "ltdc.h"
#include "quadspi.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_core.h"
#include "usbh_msc.h"
#include "gt9147.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
USBH_HandleTypeDef  hUSBHost;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	FATFS fs1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint16_t testsram[250000] __attribute__((at(0XC0000000)));//测试用数组
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void USBH_UserProcess(USBH_HandleTypeDef * phost, uint8_t id)
{
    uint32_t total,free;
	uint8_t res=0;
    switch (id)
    {
        case HOST_USER_SELECT_CONFIGURATION:
            break;
        case HOST_USER_DISCONNECTION:
            f_mount(0,"3:",1); 	//卸载U盘
//            POINT_COLOR=RED;    //设置字体为红色	   
            printf("设备连接中...\r\n");
//            LCD_Fill(30,160,239,220,WHITE); 
            break;
        case HOST_USER_CLASS_ACTIVE:
            printf("设备连接成功!\r\n");	
            res=f_mount(&fs1,"3:",1); 	//重新挂载U盘
            res=exf_getfree("3:",&total,&free);
        	if(res==0)
            {
				printf("U盘挂载成功,total=%d,free=%d\r\n",total,free);
            }
            else
            {
                printf("U盘存储空间获取失败\r\n");
            }
            break;
        case HOST_USER_CONNECTION:
            break;
        default:
            break;
    }
}

void USB_HOST_Init(void)
{
	USBH_Init(&hUSBHost, USBH_UserProcess, USBH_SPEED_FULL);
    USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);
    USBH_Start(&hUSBHost);
}

void test(void)
{
	FIL fil;

	uint8_t res;
	uint8_t write[19]="this is write test";
	uint8_t read[19];
	UINT wr_bw;
	UINT r_bw;
	switch(Key_Scan(0))
	{
	  case KEY0_PRES:
		 
		  res=f_open(&fil,"3:abcd.txt",FA_CREATE_ALWAYS|FA_WRITE);
			if(res)
			{
				printf("open error %d\r\n",res);
			}
			else
			{
				printf("open success\r\n");
			}
		  printf("key0 is press\r\n");
	  break;
	  case KEY1_PRES:
		   res=f_write(&fil,write,19,&wr_bw);
			if(res)
			{
				printf("write error %d\r\n",res);
			}
			else
			{
				printf("write success\r\n");
			}
		  printf("key1 is press\r\n");
	  break;
	  case KEY2_PRES:
		  res=f_close(&fil);
			if(res)
			{
				printf("close error %d\r\n",res);
			}
			else
			{
				printf("close success\r\n");
			}
		  printf("key2 is press\r\n");
	  break;
	  case WKUP_PRES:
		  res=f_read(&fil,read,19,&r_bw);
			if(res)
			{
				printf("read error %d\r\n",res);
				LTDC_ShowString(0,32,32,"error\r\n");
			}
			else
			{
				printf("read success,size=%d\r\n",r_bw);
				LTDC_ShowString(0,32,32,read);
				
			}
			
		  printf("wk_up is press\r\n");
	  break;
	  default : break;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t res=0;	  
	uint8_t key;
	uint8_t chinese_ok=0;
  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_SDMMC1_SD_Init();
  /* USER CODE BEGIN 2 */
	W25QXX_Init();					//SPI FLASH初始化
	delay_init(216);				//延时函数初始化，时钟频率216M
	PCF8574_Init();				    //初始化PCF8574 
	read_sdinfo();					//获取SD卡信息
    SDRAM_Init();					//初始化SDRAM
	USB_HOST_Init();				//USB
//	MPU_Memory_Protection();		//MPU内存保护配置					
    LTDC_LCD_Init();
	GT9147_Init();
	my_mem_init(SRAMIN);		    //初始化内部内存池
	my_mem_init(SRAMEX);		    //初始化外部内存池
	my_mem_init(SRAMDTCM);		    //初始化DTCM内存池
	HAL_UART_Receive_IT(&huart1,aRecBuff,1);
    HAL_TIM_Base_Start_IT(&htim3);
	
//	LTDC_ShowString(100,0,32,"F7 TEST");
	exfuns_init();
	res=f_mount(fs[0],"0:",1);//挂载SD卡
	
	if(font_init()) 		       			//检查字库
	{
	UPD:
		chinese_ok=0;
		if(res)
		{
			LTDC_ShowString(30,70,16,"SD Mount Failed!");
			HAL_Delay(200);
			LTDC_LCD_Fill(30,70,200+30,70+16,WHITE);
			HAL_Delay(200);	
		}
		else
		{
			key=update_font(20,110,16,"0:");//更新字库
			if(key)
			{
				LTDC_ShowString(30,110,16,"Font Update Failed!");
				HAL_Delay(200);
				LTDC_LCD_Fill(20,110,200+20,110+16,WHITE);
				HAL_Delay(200);		
			}
			else
			{
				chinese_ok=1;
			}
		}
	}
	else
	{
		chinese_ok=1;
		POINT_COLOR=RED;       
		Show_Str(30,30,200,16,"阿波罗STM32H7开发板",16,0);				    	 
		Show_Str(30,50,200,16,"GBK字库测试程序",16,0);				    	 
		Show_Str(30,70,200,16,"正点原子@ALIENTEK",16,0);				    	 
		Show_Str(30,90,200,16,"2017年8月16日",16,0);
		Show_Str(30,110,200,16,"按KEY0,更新字库",16,0);
		POINT_COLOR=BLUE;  
		Show_Str(30,130,200,16,"内码高字节:",16,0);				    	 
		Show_Str(30,150,200,16,"内码低字节:",16,0);				    	 
		Show_Str(30,170,200,16,"汉字计数器:",16,0);

		Show_Str(30,200,200,32,"对应汉字为:",32,0); 
		Show_Str(30,232,200,24,"对应汉字为:",24,0); 
		Show_Str(30,256,200,16,"对应汉字(16*16)为:",16,0);			 
	}
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  GT9147_Scan(0);
//	  ctp_test();
//	   USBH_Process(&hUSBHost);
		key=Key_Scan(0);
		if(key==KEY0_PRES)goto UPD;
	  if(chinese_ok==1)
	  {
		Show_Str(30,272,200,16,"韩帅鹏",12,0);
	  }
//	  Get_KeyVul();
      HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 432;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 6;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
