/******************** (C) COPYRIGHT 2012 WildFire Team ***************************
 *           ----------------- 
 * 库版本  ：ST3.5.0

**********************************************************************************/
#include "led.h"
unsigned char testxx=0;
/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 */
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	 
 	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
// ????????? GPIO_Remap_SWJ_Disable SWJ ????(JTAG+SW-DP)
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
// ????????? GPIO_Remap_SWJ_JTAGDisable ,JTAG-DP ?? + SW-DP ??
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	 
		
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;	/*选择要控制的GPIOC引脚*/	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	/*设置引脚速率为50MHz */  	
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	/*调用库函数，初始化GPIOA*/
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;	/*选择要控制的GPIOC引脚*/	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	/*设置引脚速率为50MHz */  	
  	GPIO_Init(GPIOE, &GPIO_InitStructure);	/*调用库函数，初始化GPIOA*/
		GPIO_SetBits(GPIOE,GPIO_Pin_5);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //BEEP-->GPIOB.8 ????
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //????
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //??? 50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure); //??????? GPIOB.8
 GPIO_ResetBits(GPIOB,GPIO_Pin_8); //?? 0,???????
		
}





/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
