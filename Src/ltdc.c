/**
  ******************************************************************************
  * File Name          : LTDC.c
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

/* Includes ------------------------------------------------------------------*/
#include "ltdc.h"

/* USER CODE BEGIN 0 */
uint16_t ltdc_lcd_framebuf[1280][800] __attribute__((at(LCD_FRAME_BUF_ADDR)));	//定义最大屏分辨率时,LCD所需的帧缓存数组大小
uint32_t *ltdc_framebuf[2];					//LTDC LCD帧缓存数组指针,必须指向对应大小的内存区域
_ltdc_dev lcdltdc;						    //管理LCD LTDC的重要参数
/* USER CODE END 0 */

LTDC_HandleTypeDef hltdc;

/* LTDC init function */
void MX_LTDC_Init(void)
{
  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 0;
  hltdc.Init.VerticalSync = 0;
  hltdc.Init.AccumulatedHBP = 40;
  hltdc.Init.AccumulatedVBP = 8;
  hltdc.Init.AccumulatedActiveW = 520;
  hltdc.Init.AccumulatedActiveH = 280;
  hltdc.Init.TotalWidth = 525;
  hltdc.Init.TotalHeigh = 288;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 0xff;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xC0000000 ;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_LTDC_MspInit(LTDC_HandleTypeDef* ltdcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(ltdcHandle->Instance==LTDC)
  {
  /* USER CODE BEGIN LTDC_MspInit 0 */

  /* USER CODE END LTDC_MspInit 0 */
    /* LTDC clock enable */
    __HAL_RCC_LTDC_CLK_ENABLE();
  
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**LTDC GPIO Configuration    
    PI9     ------> LTDC_VSYNC
    PI10     ------> LTDC_HSYNC
    PF10     ------> LTDC_DE
    PA5     ------> LTDC_R4
    PH9     ------> LTDC_R3
    PH11     ------> LTDC_R5
    PH12     ------> LTDC_R6
    PG6     ------> LTDC_R7
    PG7     ------> LTDC_CLK
    PH13     ------> LTDC_G2
    PH14     ------> LTDC_G3
    PH15     ------> LTDC_G4
    PI0     ------> LTDC_G5
    PI1     ------> LTDC_G6
    PI2     ------> LTDC_G7
    PG11     ------> LTDC_B3
    PI4     ------> LTDC_B4
    PI5     ------> LTDC_B5
    PI6     ------> LTDC_B6
    PI7     ------> LTDC_B7 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_0|GPIO_PIN_1 
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN LTDC_MspInit 1 */

  /* USER CODE END LTDC_MspInit 1 */
  }
}

void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* ltdcHandle)
{

  if(ltdcHandle->Instance==LTDC)
  {
  /* USER CODE BEGIN LTDC_MspDeInit 0 */

  /* USER CODE END LTDC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LTDC_CLK_DISABLE();
  
    /**LTDC GPIO Configuration    
    PI9     ------> LTDC_VSYNC
    PI10     ------> LTDC_HSYNC
    PF10     ------> LTDC_DE
    PA5     ------> LTDC_R4
    PH9     ------> LTDC_R3
    PH11     ------> LTDC_R5
    PH12     ------> LTDC_R6
    PG6     ------> LTDC_R7
    PG7     ------> LTDC_CLK
    PH13     ------> LTDC_G2
    PH14     ------> LTDC_G3
    PH15     ------> LTDC_G4
    PI0     ------> LTDC_G5
    PI1     ------> LTDC_G6
    PI2     ------> LTDC_G7
    PG11     ------> LTDC_B3
    PI4     ------> LTDC_B4
    PI5     ------> LTDC_B5
    PI6     ------> LTDC_B6
    PI7     ------> LTDC_B7 
    */
    HAL_GPIO_DeInit(GPIOI, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_0|GPIO_PIN_1 
                          |GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11);

  /* USER CODE BEGIN LTDC_MspDeInit 1 */

  /* USER CODE END LTDC_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void LTDC_LCD_Init(void)
{
    lcdltdc.pwidth=480;			    //面板宽度,单位:像素
    lcdltdc.pheight=272;		    //面板高度,单位:像素
    lcdltdc.hsw=1;				    //水平同步宽度
    lcdltdc.vsw=1;				    //垂直同步宽度
    lcdltdc.hbp=40;				    //水平后廊
    lcdltdc.vbp=8;				    //垂直后廊
    lcdltdc.hfp=5;				    //水平前廊
    lcdltdc.vfp=8;				    //垂直前廊
    lcdltdc.pixsize=2;				//每个像素占2个字节
	ltdc_framebuf[0]=(uint32_t*)&ltdc_lcd_framebuf;
	LTDC_Select_Layer(0);
	LTDC_Layer_Window_Config(0,0,0,lcdltdc.pwidth,lcdltdc.pheight);	//层窗口配置,以LCD面板坐标系为基准,不要随便修改!
    LTDC_Display_Dir(0);//横屏
	LTDC_LCD_ON;//lcd背光
    LTDC_LCD_Clear(WHITE);
}
//选择层
//layerx:层号;0,第一层;1,第二层;
void LTDC_Select_Layer(uint8_t layerx)
{
	lcdltdc.activelayer=layerx;
}
/************************************************************************************************
函数功能：LTDC,层颜窗口设置,窗口以LCD面板坐标系为基准
入口参数：layerx:层值,0/1.
			sx,sy:起始坐标
			width,height:宽度和高度
返回值：无
注意:此函数必须在LTDC_Layer_Parameter_Config之后再设置.
************************************************************************************************/
void LTDC_Layer_Window_Config(uint8_t layerx,uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
    HAL_LTDC_SetWindowPosition(&hltdc,sx,sy,layerx);  //设置窗口的位置
    HAL_LTDC_SetWindowSize(&hltdc,width,height,layerx);//设置窗口大小    
}
/************************************************************************************************
函数功能：设置LCD显示方向
入口参数：dir:0,竖屏；1,横屏
返回值：无
************************************************************************************************/
void LTDC_Display_Dir(uint8_t dir)
{
    lcdltdc.dir=dir; 	//显示方向
	if(dir==0)			//竖屏
	{
		lcdltdc.width=lcdltdc.pheight;
		lcdltdc.height=lcdltdc.pwidth;	
	}else if(dir==1)	//横屏
	{
		lcdltdc.width=lcdltdc.pwidth;
		lcdltdc.height=lcdltdc.pheight;
	}
}
/************************************************************************************************
函数功能：指定位置画点
入口参数：(x,y)，颜色
返回值：无
横屏扫描方向为从左到右，从上到下，显存首地址和坐标原点都在左上角
(0,0)  ----------------
 buff0 | →→→→→→→→→→→→→|
       |↓             |
       |↓             |
       |↓             |
       |↓             |
       ----------------
横屏时每行480个点，y行为480*y，此为要显示的行坐标。 x即为显示的列坐标。则在显存的位置为480*y+x  
-----------------------------------------------
竖屏时扫描方向为从右到左，从上到下,(横屏向右旋转)
(0,0)      buff首地址
|-------↓↓|
|--------↓|
|--------↓|
|--------↓|
|--------↓|
|--------↓|
|--------↓|
竖屏时，480*(272-x)，即为要显示的行坐标，在显存的位置为480*(272-x)+y
************************************************************************************************/
void LTDC_Draw_Point(uint16_t x,uint16_t y,uint32_t color)
{
	if( lcdltdc.dir)//横屏
	{                                                                                          
		*(uint16_t *)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*y+x))=color;
	}
	else                                                                                        
	{                                                                                    
		*(uint16_t *)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*(lcdltdc.pheight-x)+y))=color;
	}
}
/************************************************************************************************
函数功能：读点函数
入口参数：x,y:读取点的坐标
返回值：颜色值
************************************************************************************************/
uint32_t LTDC_Read_Point(uint16_t x,uint16_t y)
{ 
	if(lcdltdc.dir)	//横屏
	{
		return *(uint16_t*)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*y+x));
	}else 			//竖屏
	{
		return *(uint16_t*)((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*(lcdltdc.pheight-x-1)+y)); 
	}
}
/**************************************************************************************
函数功能：LTDC填充矩形,DMA2D填充
入口参数：(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
			color:要填充的颜色
返回值：无
有时候需要频繁的调用填充函数，所以为了速度，填充函数采用寄存器版本，
***************************************************************************************/
void LTDC_LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint32_t color)
{
	uint32_t psx,psy,pex,pey;	//以LCD面板为基准的坐标系,不随横竖屏变化而变化
	uint32_t timeout=0; 
	uint16_t offline;
	uint32_t addr; 
	//坐标系转换
	if(lcdltdc.dir)	//横屏
	{
		psx=sx;psy=sy;
		pex=ex;pey=ey;
	}else			//竖屏
	{
		psx=sy;psy=lcdltdc.pheight-ex-1;
		pex=ey;pey=lcdltdc.pheight-sx-1;
	}
	offline=lcdltdc.pwidth-(pex-psx+1);
	addr=((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*psy+psx));
	__HAL_RCC_DMA2D_CLK_ENABLE();	//使能DM2D时钟
	DMA2D->CR&=~(DMA2D_CR_START);	//先停止DMA2D
	DMA2D->CR=DMA2D_R2M;			//寄存器到存储器模式
	DMA2D->OPFCCR=LTDC_PIXEL_FORMAT_RGB565;	//设置颜色格式
	DMA2D->OOR=offline;				//设置行偏移 

	DMA2D->OMAR=addr;				//输出存储器地址
	DMA2D->NLR=(pey-psy+1)|((pex-psx+1)<<16);	//设定行数寄存器
	DMA2D->OCOLR=color;						//设定输出颜色寄存器 
	DMA2D->CR|=DMA2D_CR_START;				//启动DMA2D
	while((DMA2D->ISR&(DMA2D_FLAG_TC))==0)	//等待传输完成
	{
		timeout++;
		if(timeout>0X1FFFFF)break;	//超时退出
	} 
	DMA2D->IFCR|=DMA2D_FLAG_TC;		//清除传输完成标志 		
}
/****************************************************************************
函数功能：在指定区域内填充指定颜色块,DMA2D填充
入口参数：(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)   
			color:要填充的颜色数组首地址
返回值：无
注意:sx,ex,不能大于lcddev.width-1;sy,ey,不能大于lcddev.height-1!!!
有时候需要频繁的调用填充函数，所以为了速度，填充函数采用寄存器版本，
*****************************************************************************/
void LTDC_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color)
{
	uint32_t psx,psy,pex,pey;	//以LCD面板为基准的坐标系,不随横竖屏变化而变化
	uint32_t timeout=0; 
	uint16_t offline;
	uint32_t addr; 
	//坐标系转换
	if(lcdltdc.dir)	//横屏
	{
		psx=sx;psy=sy;
		pex=ex;pey=ey;
	}else			//竖屏
	{
		psx=sy;psy=lcdltdc.pheight-ex-1;
		pex=ey;pey=lcdltdc.pheight-sx-1;
	}
	offline=lcdltdc.pwidth-(pex-psx+1);
	addr=((uint32_t)ltdc_framebuf[lcdltdc.activelayer]+lcdltdc.pixsize*(lcdltdc.pwidth*psy+psx));
	__HAL_RCC_DMA2D_CLK_ENABLE();	//使能DM2D时钟
	DMA2D->CR&=~(DMA2D_CR_START);	//先停止DMA2D
	DMA2D->CR=DMA2D_M2M;			//存储器到存储器模式
	DMA2D->FGPFCCR=LTDC_PIXEL_FORMAT_RGB565;	//设置颜色格式
	DMA2D->FGOR=0;					//前景层行偏移为0
	DMA2D->OOR=offline;				//设置行偏移 

	DMA2D->FGMAR=(uint32_t)color;		//源地址
	DMA2D->OMAR=addr;				//输出存储器地址
	DMA2D->NLR=(pey-psy+1)|((pex-psx+1)<<16);	//设定行数寄存器 
	DMA2D->CR|=DMA2D_CR_START;					//启动DMA2D
	while((DMA2D->ISR&(DMA2D_FLAG_TC))==0)		//等待传输完成
	{
		timeout++;
		if(timeout>0X1FFFFF)break;	//超时退出
	} 
	DMA2D->IFCR|=DMA2D_FLAG_TC;				//清除传输完成标志  	
}
/************************************************
函数功能：清屏
入口参数：color:要填充的颜色
返回值：无
*************************************************/
void LTDC_LCD_Clear(uint32_t color)
{
	LTDC_LCD_Fill(0,0,lcdltdc.width-1,lcdltdc.height-1,color);
    BACK_COLOR=color;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
