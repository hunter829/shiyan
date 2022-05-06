#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK MiniSTM32������
//STM32 FLASH ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/12
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////	
//********************************************************************************
//V1.1�޸�˵��
//������STMFLASH_Write������ַƫ�Ƶ�һ��bug.
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
 struct para_type{  
	      int  work_NO;
	      char  board_ID[20];
        char  log_id[10];     ///  ���еĵ������ ��ÿ16��һ��
       
        char input1_old;
        char input2_old; 
        char input3_old;	 
	      int  power_off_n;
	      int  position;	
	
	      
	      u32   soc_pos;
				u32   xxxx;
				 
	
};    
 struct SOC_type{  
         
	      u32   SOC;
				u32 SOC_n;
	      float SOC_AH;
	      u32   xxxx;  
	 
	
};    

#define StartAddr_bmu_para   ((u32)0x08008000)
#define StartAddr_soc_para   ((u32)0x08008800)
 void STMFLASH_Read_byte_n(u32 ReadAddr,u8 *pBuffer,u16 NumToRead);






//#define FLASH_EEPROM		//����ʹ���ڲ�FLASHģ��EEPROMʹ��
#define FLASH_SAVE_ADDR  0X08020000 	//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)

/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE    ((u16)0x400)   //USE_STM3210B
#define StartAddr          ((u32)0x08008000)
#define EndAddr            ((u32)0x0800C000)

//�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 	64 	 			//��ѡSTM32��FLASH������С(��λΪK)
#define STM32_FLASH_WREN 	1              		//ʹ��FLASHд��(0��������;1��ʹ��)
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define FLASH_SAVE_ADDR  0X08020000 	//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 			//STM32 FLASH����ʼ��ַ
//FLASH������ֵ
#define FLASH_KEY1               0X45670123
#define FLASH_KEY2               0XCDEF89AB
void STMFLASH_Unlock(void);					  	//FLASH����
void STMFLASH_Lock(void);					  	//FLASH����
u8 STMFLASH_GetStatus(void);				  	//���״̬
u8 STMFLASH_WaitDone(u16 time);				  	//�ȴ���������
u8 STMFLASH_ErasePage(u32 paddr);			  	//����ҳ
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);	//д�����
u16 STMFLASH_ReadHalfWord(u32 faddr);		  	//��������  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(u32 WriteAddr,u16 WriteData);								   
#endif

















