/**
  ******************************************************************************
  * File Name          : LTDC.h
  * Description        : This file provides code for the configuration
  *                      of the LTDC instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ltdc_H
#define __ltdc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern LTDC_HandleTypeDef hltdc;

/* USER CODE BEGIN Private defines */
//LCD帧缓冲区首地址,这里定义在SDRAM里面.
#define LCD_FRAME_BUF_ADDR			0XC0000000  
#define LTDC_LCD_ON					HAL_GPIO_WritePin(LCD_BL_GPIO_Port,LCD_BL_Pin,GPIO_PIN_SET);
#define LTDC_LCD_OFF				HAL_GPIO_WritePin(LCD_BL_GPIO_Port,LCD_BL_Pin,GPIO_PIN_RESET);	
//LCD LTDC重要参数集
typedef struct  
{							 
	uint32_t pwidth;			//LCD面板的宽度,固定参数,不随显示方向改变,如果为0,说明没有任何RGB屏接入
	uint32_t pheight;		//LCD面板的高度,固定参数,不随显示方向改变
	uint16_t hsw;			//水平同步宽度
	uint16_t vsw;			//垂直同步宽度
	uint16_t hbp;			//水平后廊
	uint16_t vbp;			//垂直后廊
	uint16_t hfp;			//水平前廊
	uint16_t vfp;			//垂直前廊 
	uint8_t activelayer;		//当前层编号:0/1	
	uint8_t dir;				//0,竖屏;1,横屏;
	uint16_t width;			//LCD宽度
	uint16_t height;			//LCD高度
	uint32_t pixsize;		//每个像素所占字节数
}_ltdc_dev; 
extern _ltdc_dev lcdltdc;		            //管理LCD LTDC参数
/* USER CODE END Private defines */

void MX_LTDC_Init(void);

/* USER CODE BEGIN Prototypes */
void LTDC_LCD_Init(void);
void LTDC_Select_Layer(uint8_t layerx);
void LTDC_Layer_Window_Config(uint8_t layerx,uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);
void LTDC_Display_Dir(uint8_t dir);
void LTDC_Draw_Point(uint16_t x,uint16_t y,uint32_t color);
uint32_t LTDC_Read_Point(uint16_t x,uint16_t y);
void LTDC_LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint32_t color);
void LTDC_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);
void LTDC_LCD_Clear(uint32_t color);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ ltdc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
