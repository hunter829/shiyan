/******************** (C) COPYRIGHT 2012 WildFire Team ***************************
 *           ----------------- 
 * ��汾  ��ST3.5.0

**********************************************************************************/
#include "led.h"
unsigned char testxx=0;
/*
 * ��������LED_GPIO_Config
 * ����  ������LED�õ���I/O��
 */
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	 
 	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
// ????????? GPIO_Remap_SWJ_Disable SWJ ????(JTAG+SW-DP)
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
// ????????? GPIO_Remap_SWJ_JTAGDisable ,JTAG-DP ?? + SW-DP ??
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	 
		
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;	/*ѡ��Ҫ���Ƶ�GPIOC����*/	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	/*������������Ϊ50MHz */  	
  	GPIO_Init(GPIOB, &GPIO_InitStructure);	/*���ÿ⺯������ʼ��GPIOA*/
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;	/*ѡ��Ҫ���Ƶ�GPIOC����*/	
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	/*������������Ϊ50MHz */  	
  	GPIO_Init(GPIOE, &GPIO_InitStructure);	/*���ÿ⺯������ʼ��GPIOA*/
		GPIO_SetBits(GPIOE,GPIO_Pin_5);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //BEEP-->GPIOB.8 ????
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //????
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //??? 50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure); //??????? GPIOB.8
 GPIO_ResetBits(GPIOB,GPIO_Pin_8); //?? 0,???????
		
}





/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
