#include "ALL_Includes.h"




#define DATA_DIR  P4DIR      
#define DATA_OUT  P4OUT




void LcdGpioConfig(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9| GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

}
INT16U Iodata[8]=
{
	GPIO_Pin_12,
	GPIO_Pin_2,
	GPIO_Pin_1,
	GPIO_Pin_5,
	GPIO_Pin_6,
	GPIO_Pin_7,
	GPIO_Pin_8,
	GPIO_Pin_9
};

void  write_InData(INT8U Data)
{
	INT8U i;
	for(i=0;i<8;i++)
	{
		if(Data&0x01)
		{
		 GPIO_SetBits(GPIOB,Iodata[i]); 
		}else
		{
		 GPIO_ResetBits(GPIOB,Iodata[i]);
		}
		Data>>=1;
	
	}
    

}

/////////////////写命令///////////////
void  write_com(uchar com)
{ 
	GPIO_ResetBits(GPIOB,GPIO_Pin_15); //RS=0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_14); //RW=0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_13); //EN=0;
	write_InData(com);
	delay_us(300);
	GPIO_SetBits(GPIOB,GPIO_Pin_13); //EN=1;
	delay_us(300);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13); //EN=0;
	delay_us(300);
}
////////////////写数据///////////////
void  write_data(uchar dat)
{  	
	GPIO_SetBits(GPIOB,GPIO_Pin_15); //RS=1;
	GPIO_ResetBits(GPIOB,GPIO_Pin_14); //RW=0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_13); //EN=0;
	write_InData(dat);
	delay_us(300);
	GPIO_SetBits(GPIOB,GPIO_Pin_13); //EN=1;
	delay_us(300);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13); //EN=0;
	delay_us(300);
}
/////////////////液晶初始化/////////////
void Init_lcd(void)
{
    delay_ms(5);
    write_com(0x38);
    delay_ms(5);
    write_com(0x38);
    delay_ms(5);
    write_com(0x38);
    write_com(0x38);
    write_com(0x08);
    write_com(0x01);
    write_com(0x06);
    write_com(0x0c);
}
static INT32U HexToAscii(INT8U *data, INT8U *buffer, INT32U len)
{
	const INT8U ascTable[17] = {"0123456789ABCDEF"};
	INT8U *tmp_p = buffer;
	INT32U i, pos;
	pos = 0;
	for(i = 0; i < len; i++)
	{
	tmp_p[pos++] = ascTable[data[i] >> 4];
	tmp_p[pos++] = ascTable[data[i] & 0x0f];
	}
	tmp_p[pos] = '\0';
	return pos;
} 


void display(INT8U *buff,INT8U Len)//显示函数
{
	INT8U DisBuff[32],i;
	
	HexToAscii(buff,DisBuff,Len);
   for(i=0;i<Len*2;i++)
   {
	   if(i<16)	
	   {   
	   	write_com(0x80+i);//显示
	   }else
	   {
	     write_com(0xC0+i-16);//显示
	   }
	   write_data(DisBuff[i]);
   }

}
