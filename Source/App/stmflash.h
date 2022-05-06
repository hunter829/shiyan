#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK MiniSTM32开发板
//STM32 FLASH 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/12
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////	
//********************************************************************************
//V1.1修改说明
//修正了STMFLASH_Write函数地址偏移的一个bug.
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
 struct para_type{  
	      int  work_NO;
	      char  board_ID[20];
        char  log_id[10];     ///  所有的电池配置 ，每16节一个
       
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






//#define FLASH_EEPROM		//允许使用内部FLASH模拟EEPROM使用
#define FLASH_SAVE_ADDR  0X08020000 	//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

/* Private define ------------------------------------------------------------*/
#define FLASH_PAGE_SIZE    ((u16)0x400)   //USE_STM3210B
#define StartAddr          ((u32)0x08008000)
#define EndAddr            ((u32)0x0800C000)

//用户根据自己的需要设置
#define STM32_FLASH_SIZE 	64 	 			//所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_WREN 	1              		//使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define FLASH_SAVE_ADDR  0X08020000 	//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 			//STM32 FLASH的起始地址
//FLASH解锁键值
#define FLASH_KEY1               0X45670123
#define FLASH_KEY2               0XCDEF89AB
void STMFLASH_Unlock(void);					  	//FLASH解锁
void STMFLASH_Lock(void);					  	//FLASH上锁
u8 STMFLASH_GetStatus(void);				  	//获得状态
u8 STMFLASH_WaitDone(u16 time);				  	//等待操作结束
u8 STMFLASH_ErasePage(u32 paddr);			  	//擦除页
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);	//写入半字
u16 STMFLASH_ReadHalfWord(u32 faddr);		  	//读出半字  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//指定地址开始写入指定长度的数据
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//指定地址开始读取指定长度数据
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据

//测试写入
void Test_Write(u32 WriteAddr,u16 WriteData);								   
#endif

















