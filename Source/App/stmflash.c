#include "stmflash.h"
#include "delay.h"
//#include "usart.h"
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


#include "stdio.h"
 #include "uart.h"




//解锁STM32的FLASH
void STMFLASH_Unlock(void)
{
  FLASH->KEYR=FLASH_KEY1;//写入解锁序列.
  FLASH->KEYR=FLASH_KEY2;
}
//flash上锁
void STMFLASH_Lock(void)
{
  FLASH->CR|=1<<7;//上锁
}
//得到FLASH状态
u8 STMFLASH_GetStatus(void)
{	
	u32 res;		
	res=FLASH->SR; 
	if(res&(1<<0))return 1;		    //忙
	else if(res&(1<<2))return 2;	//编程错误
	else if(res&(1<<4))return 3;	//写保护错误
	return 0;						//操作完成
}
//等待操作完成
//time:要延时的长短
//返回值:状态.
u8 STMFLASH_WaitDone(u16 time)
{
	u8 res;
	do
	{
		res=STMFLASH_GetStatus();
		if(res!=1)break;//非忙,无需等待了,直接退出.
		delay_us(1);
		time--;
	 }while(time);
	 if(time==0)res=0xff;//TIMEOUT
	 return res;
}
//擦除页
//paddr:页地址
//返回值:执行情况
u8 STMFLASH_ErasePage(u32 paddr)
{
	u8 res=0;
	res=STMFLASH_WaitDone(0X5FFF);//等待上次操作结束,>20ms    
	if(res==0)
	{ 
		FLASH->CR|=1<<1;//页擦除
		FLASH->AR=paddr;//设置页地址 
		FLASH->CR|=1<<6;//开始擦除		  
		res=STMFLASH_WaitDone(0X5FFF);//等待操作结束,>20ms  
		if(res!=1)//非忙
		{
			FLASH->CR&=~(1<<1);//清除页擦除标志.
		}
	}
	return res;
}
//在FLASH指定地址写入半字
//faddr:指定地址(此地址必须为2的倍数!!)
//dat:要写入的数据
//返回值:写入的情况
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat)
{
	u8 res;	   	    
	res=STMFLASH_WaitDone(0XFF);	 
	if(res==0)//OK
	{
		FLASH->CR|=1<<0;//编程使能
		*(vu16*)faddr=dat;//写入数据
		res=STMFLASH_WaitDone(0XFF);//等待操作完成
		if(res!=1)//操作成功
		{
			FLASH->CR&=~(1<<0);//清除PG位.
		}
	} 
	return res;
} 
//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}

u8 STMFLASH_Read_byte(u32 faddr)
{
	return *(vu8*)faddr; 
}

#if STM32_FLASH_WREN	//如果使能了写   
//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		STMFLASH_WriteHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//地址增加2.
	}  
} 
//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //扇区地址
	u16 secoff;	   //扇区内偏移地址(16位字计算)
	u16 secremain; //扇区内剩余地址(16位字计算)	   
 	u16 i;    
	u32 offaddr;   //去掉0X08000000后的地址
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
	STMFLASH_Unlock();						//解锁
	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
	secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
	secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小   
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容
		for(i=0;i<secremain;i++)//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除  	  
		}
		if(i<secremain)//需要擦除
		{
			STMFLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区  
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 				   
		if(NumToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
		   	pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain*2;	//写地址偏移(16位数据地址,需要*2)	   
		   	NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	 
	};	
	STMFLASH_Lock();//上锁
}
#endif
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
	}
}
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read_byte_n(u32 ReadAddr,u8 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_Read_byte(ReadAddr);//读取2个字节.
		ReadAddr++;//偏移2个字节.	
	}
}
//////////////////////////////////////////测试用///////////////////////////////////////////
//WriteAddr:起始地址
//WriteData:要写入的数据
void Test_Write(u32 WriteAddr,u16 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//写入一个字 
}



extern struct para_type *my_para;
extern  struct para_type my_para_real;
extern   struct SOC_type *my_soc;
extern  struct SOC_type my_soc_real;
 #define bmu_para_size   (sizeof(my_para_real)+10)     //  256 byte
#define soc_para_size    (sizeof(my_soc_real)+10)     //  256 byte

unsigned int para_checksum=0xff,para_checksum_read;
extern unsigned char my_addr;

void read_my_para()
{
STMFLASH_Read(StartAddr_bmu_para,(u16*)(my_para),bmu_para_size); 

}



void reinit_para(void)
{
	int i=0;

	return;
	for(i=0;i<10;i++)
	{
		my_para_real.board_ID [i]='0';
		my_para_real.log_id  [i]='0';
		
	}
 
	
   my_para_real.soc_pos=	StartAddr_soc_para;
//  my_para_real.max_charge_current=50;
//	my_para_real.need_chager_voletage=2400;
	
}
extern char message[100];
void para_setting()
{
///////////////////////////////////////  para test
	int i=0;
	 sprintf(message,"@para size! %d #\r\n" ,bmu_para_size    );				 
			//   uart1_send_str(message);
	 
   my_para=&my_para_real;

	STMFLASH_Read(StartAddr_bmu_para,(u16*)(my_para),bmu_para_size); 
					 
	  
		sprintf(message,"@%d,para show#\r\n",my_addr     );				 
			     // uart1_send_str(message);				
			 
	     
	////////////////////////////////////  para test end		
}
void para_save()
{
	int i=0;
		para_checksum=0xff;		 
	for(i=0;i<bmu_para_size-1;i++)
	{
	 para_checksum=para_checksum^(*((u16*)(my_para)+i));
	}
	 (*((u16*)(my_para)+bmu_para_size-1)) =para_checksum;
	
 STMFLASH_Write(StartAddr_bmu_para,(u16*)my_para,bmu_para_size); 
			
}

void SOC_save()
{
	int i=0;
		para_checksum=0xff;		 
	for(i=0;i<soc_para_size-1;i++)
	{
	 para_checksum=para_checksum^(*((u16*)(my_soc)+i));
	}
	 (*((u16*)(my_soc)+soc_para_size-1)) =para_checksum;
	
 STMFLASH_Write(my_para->soc_pos,(u16*)my_soc,soc_para_size); 
		//	sprintf(message,"@checksum   %d,  #\r\n" ,para_checksum    );				 
		//	   uart1_send_str(message);
}


void reinit_soc(void)
{
	int i=0;
 
	 my_soc->SOC=0;
	 my_soc->SOC_AH=0;
	 my_soc->SOC_n=0;
   
//  my_para_real.max_charge_current=50;
//	my_para_real.need_chager_voletage=2400;
	
}
void soc_setting()
{
///////////////////////////////////////  para test
	int i=0;
	 sprintf(message,"@soc size! %d #\r\n" ,soc_para_size    );				 
			 //  uart1_send_str(message);
	
	
	
	
   my_soc=&my_soc_real;

	STMFLASH_Read(my_para->soc_pos,(u16*)(my_soc),soc_para_size); 
					 para_checksum=0xff;	
	for(i=0;i<soc_para_size-1;i++)
	{
	 para_checksum=para_checksum^(*((u16*)(my_soc)+i));
	}
	para_checksum_read=(*((u16*)(my_soc)+soc_para_size-1)) ;
	
		 
	if(para_checksum!=para_checksum_read )
	{
	   //reinit_soc();
		para_checksum=0xff;
		for(i=0;i<soc_para_size-1;i++)
	   {
	    para_checksum=para_checksum^(*((u16*)(my_soc)+i));
	   }
		(*((u16*)(my_soc)+soc_para_size-1)) =para_checksum;
		
		STMFLASH_Write(my_para->soc_pos,(u16*)my_soc,soc_para_size); 
		 
			 sprintf(message,"@checksum error reinit and save! %d,%d #\r\n" ,para_checksum,para_checksum_read    );				 
			  // uart1_send_str(message);
	}
	
		sprintf(message,"@%d,soc show#\r\n",my_addr     );				 
			     // uart1_send_str(message);				
			 
			 
			  
			 sprintf(message,"@SOC_n %d#\r\n", my_soc->SOC_n    );				 
			    //  uart1_send_str(message);		
			 
       sprintf(message,"@SOC_AH %.4f#\r\n", my_soc->SOC_AH    );				 
			     // uart1_send_str(message);
			sprintf(message,"@soc addr %d#\r\n", my_para->soc_pos     );				 
			     // uart1_send_str(message);
			
				 
}





