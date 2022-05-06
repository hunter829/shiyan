#include "stmflash.h"
#include "delay.h"
//#include "usart.h"
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


#include "stdio.h"
 #include "uart.h"




//����STM32��FLASH
void STMFLASH_Unlock(void)
{
  FLASH->KEYR=FLASH_KEY1;//д���������.
  FLASH->KEYR=FLASH_KEY2;
}
//flash����
void STMFLASH_Lock(void)
{
  FLASH->CR|=1<<7;//����
}
//�õ�FLASH״̬
u8 STMFLASH_GetStatus(void)
{	
	u32 res;		
	res=FLASH->SR; 
	if(res&(1<<0))return 1;		    //æ
	else if(res&(1<<2))return 2;	//��̴���
	else if(res&(1<<4))return 3;	//д��������
	return 0;						//�������
}
//�ȴ��������
//time:Ҫ��ʱ�ĳ���
//����ֵ:״̬.
u8 STMFLASH_WaitDone(u16 time)
{
	u8 res;
	do
	{
		res=STMFLASH_GetStatus();
		if(res!=1)break;//��æ,����ȴ���,ֱ���˳�.
		delay_us(1);
		time--;
	 }while(time);
	 if(time==0)res=0xff;//TIMEOUT
	 return res;
}
//����ҳ
//paddr:ҳ��ַ
//����ֵ:ִ�����
u8 STMFLASH_ErasePage(u32 paddr)
{
	u8 res=0;
	res=STMFLASH_WaitDone(0X5FFF);//�ȴ��ϴβ�������,>20ms    
	if(res==0)
	{ 
		FLASH->CR|=1<<1;//ҳ����
		FLASH->AR=paddr;//����ҳ��ַ 
		FLASH->CR|=1<<6;//��ʼ����		  
		res=STMFLASH_WaitDone(0X5FFF);//�ȴ���������,>20ms  
		if(res!=1)//��æ
		{
			FLASH->CR&=~(1<<1);//���ҳ������־.
		}
	}
	return res;
}
//��FLASHָ����ַд�����
//faddr:ָ����ַ(�˵�ַ����Ϊ2�ı���!!)
//dat:Ҫд�������
//����ֵ:д������
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat)
{
	u8 res;	   	    
	res=STMFLASH_WaitDone(0XFF);	 
	if(res==0)//OK
	{
		FLASH->CR|=1<<0;//���ʹ��
		*(vu16*)faddr=dat;//д������
		res=STMFLASH_WaitDone(0XFF);//�ȴ��������
		if(res!=1)//�����ɹ�
		{
			FLASH->CR&=~(1<<0);//���PGλ.
		}
	} 
	return res;
} 
//��ȡָ����ַ�İ���(16λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}

u8 STMFLASH_Read_byte(u32 faddr)
{
	return *(vu8*)faddr; 
}

#if STM32_FLASH_WREN	//���ʹ����д   
//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��   
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		STMFLASH_WriteHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//��ַ����2.
	}  
} 
//��ָ����ַ��ʼд��ָ�����ȵ�����
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
//pBuffer:����ָ��
//NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //�ֽ�
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //������ַ
	u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
	u16 secremain; //������ʣ���ַ(16λ�ּ���)	   
 	u16 i;    
	u32 offaddr;   //ȥ��0X08000000��ĵ�ַ
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
	STMFLASH_Unlock();						//����
	offaddr=WriteAddr-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
	secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
	secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С   
	if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������
		for(i=0;i<secremain;i++)//У������
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����  	  
		}
		if(i<secremain)//��Ҫ����
		{
			STMFLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
			for(i=0;i<secremain;i++)//����
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������  
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(NumToWrite==secremain)break;//д�������
		else//д��δ����
		{
			secpos++;				//������ַ��1
			secoff=0;				//ƫ��λ��Ϊ0 	 
		   	pBuffer+=secremain;  	//ָ��ƫ��
			WriteAddr+=secremain*2;	//д��ַƫ��(16λ���ݵ�ַ,��Ҫ*2)	   
		   	NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//��һ����������д����
			else secremain=NumToWrite;//��һ����������д����
		}	 
	};	
	STMFLASH_Lock();//����
}
#endif
//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
		ReadAddr+=2;//ƫ��2���ֽ�.	
	}
}
//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Read_byte_n(u32 ReadAddr,u8 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_Read_byte(ReadAddr);//��ȡ2���ֽ�.
		ReadAddr++;//ƫ��2���ֽ�.	
	}
}
//////////////////////////////////////////������///////////////////////////////////////////
//WriteAddr:��ʼ��ַ
//WriteData:Ҫд�������
void Test_Write(u32 WriteAddr,u16 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//д��һ���� 
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





