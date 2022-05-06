/******************** STM32F103 .C File  keil 4.72 v********************************************************************
* Company			 : BDK
* File Name          : UART
* Author             : benten22  BDK
* firmware Version   : V3.5.0
* Date               : 2014/08/10
* Describe			 :

*************************************************************************************************************************/
#include "stm32f10x.h"
#include "UART.h"
#include "stdio.h"


#define USARTy                   USART1
#define USARTy_GPIO              GPIOA
#define USARTy_CLK               RCC_APB2Periph_USART1
#define USARTy_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USARTy_RxPin             GPIO_Pin_10
#define USARTy_TxPin             GPIO_Pin_9
#define USARTy_Tx_DMA_Channel    DMA1_Channel4
#define USARTy_Tx_DMA_FLAG       DMA1_FLAG_TC4
#define USARTy_DR_Base           0x40013804

#if 1
#pragma import(__use_no_semihosting)             
                
struct __FILE 				//标准库需要的支持函数 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
					  
_sys_exit(int x) 							//定义_sys_exit()以避免使用半主机模式  
{ 
	x = x; 
} 

int fputc( int ch , FILE *f )				//重定义fputc函数 
{      
	while( ( USART1->SR&0X40 ) == 0 );		//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

/* *******************************************************************************************************************/
/* ------------------------------------------------------------------------------------------------------------------*/
/* ********************************************* USART 1 *************************************************************/
/* ------------------------------------------------------------------------------------------------------------------*/
/* 1.配置基本参数*****************************************************************************************************/
void UART_RxInt( void )
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}

void UART1_Config( unsigned long Baud )
{
	USART_InitTypeDef 		USART_InitStructure; 			//UART 初始化配置结构体
	GPIO_InitTypeDef 	  	GPIO_Structure;					//GPIO 结构体配置
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE ) ;	//使能时钟
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE  );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE ); 

//引脚和时钟配置
	GPIO_Structure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Structure.GPIO_Mode = GPIO_Mode_AF_PP;
 	GPIO_Init( GPIOA , &GPIO_Structure );						//Tx复用推挽式输出
	
	GPIO_Structure.GPIO_Pin = GPIO_Pin_10;
 	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
 	GPIO_Init( GPIOA , &GPIO_Structure );						//Rx浮空输入

//UART配置
	USART_DeInit( USART1 ); 									//复位UART1
	/* UART功能设置 -----------------------------------------------------------------*/
	/*功能*/
	USART_InitStructure.USART_BaudRate = Baud; 						//设置波特率 BaudRate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //设置传输数据长度 8位或者9位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//设置停止位 1 | 0.5 | 1.5 | 2 ；
	USART_InitStructure.USART_Parity = USART_Parity_No; 			//奇偶校验设置 Odd奇 No Even偶
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None; 								//RTS , CTC , 流控制设置
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //收发使能
	USART_Init(USART1, &USART_InitStructure);						//函数实规则
	
	USART_Cmd( USART1, ENABLE );	//使能UART1
	
	USART_ClearFlag( USART1 , USART_FLAG_TXE|USART_FLAG_RXNE );
	
	USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
	UART_RxInt();
}



void UART1_WriteByte( unsigned char Wchar )
{
	while ( USART_GetFlagStatus( USART1, USART_FLAG_TXE ) != SET );
	USART_ClearFlag( USART1 , USART_FLAG_TXE );
	USART_SendData( USART1 , Wchar ); 
}


#define uart_buffer_size   100
unsigned char uart_rec_buffer[uart_buffer_size];
unsigned char index_uart=0;
unsigned char uart_buffer_head_ok;
unsigned char cmd_in=0;


void USART1_IRQHandler(void)
{
	unsigned char  c;
	 if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
	    c=USART1->DR;
		  if(c=='@') 
			{
				uart_buffer_head_ok=1;
				index_uart=0;
			}
	 	  if(uart_buffer_head_ok==1)
		    {
		     uart_rec_buffer[index_uart++]=c;
				 if(c=='#') //end ok
				 {
					 cmd_in=1;
				 }					
		    }else
				{
				 index_uart=0;
				}
		  if(index_uart>=uart_buffer_size)  index_uart=0;
		  
		
		
		// UART1_WriteByte(c); UART1_WriteByte('>'); UART1_WriteByte(c+1); 	  
	}
	
	USART_ClearITPendingBit( USART1 , USART_IT_RXNE );
}



#define USARTy2                   USART1
#define USARTy2_GPIO              GPIOA
#define USARTy2_CLK               RCC_APB2Periph_USART2
#define USARTy2_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USARTy2_RxPin             GPIO_Pin_3
#define USARTy2_TxPin             GPIO_Pin_2
#define USARTy2_Tx_DMA_Channel    DMA1_Channel4
#define USARTy2_Tx_DMA_FLAG       DMA1_FLAG_TC4
#define USARTy2_DR_Base           0x40013804
 

/* *******************************************************************************************************************/
/* ------------------------------------------------------------------------------------------------------------------*/
/* ********************************************* USART 1 *************************************************************/
/* ------------------------------------------------------------------------------------------------------------------*/
/* 1.配置基本参数*****************************************************************************************************/
void UART2_RxInt( void )
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTy2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}

void UART2_Config( unsigned long Baud )
{
	USART_InitTypeDef 		USART_InitStructure; 			//UART 初始化配置结构体
	GPIO_InitTypeDef 	  	GPIO_Structure;					//GPIO 结构体配置
	
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE); 	//使能时钟
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE  );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE ); 

//引脚和时钟配置
	GPIO_Structure.GPIO_Pin = GPIO_Pin_2;
 	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Structure.GPIO_Mode = GPIO_Mode_AF_PP;
 	GPIO_Init( GPIOA , &GPIO_Structure );						//Tx复用推挽式输出
	
	GPIO_Structure.GPIO_Pin = GPIO_Pin_3;
 	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
 	GPIO_Init( GPIOA , &GPIO_Structure );						//Rx浮空输入

//UART配置
	USART_DeInit( USART2 ); 									//复位UART1
	/* UART功能设置 -----------------------------------------------------------------*/
	/*功能*/
	USART_InitStructure.USART_BaudRate = Baud; 						//设置波特率 BaudRate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //设置传输数据长度 8位或者9位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//设置停止位 1 | 0.5 | 1.5 | 2 ；
	USART_InitStructure.USART_Parity = USART_Parity_No; 			//奇偶校验设置 Odd奇 No Even偶
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None; 								//RTS , CTC , 流控制设置
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //收发使能
	USART_Init(USART2, &USART_InitStructure);						//函数实规则
	
	USART_Cmd( USART2, ENABLE );	//使能UART1
	
	USART_ClearFlag( USART2 , USART_FLAG_TXE|USART_FLAG_RXNE );
	
	USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
	 UART2_RxInt();
}



void UART2_WriteByte( unsigned char Wchar )
{
	while ( USART_GetFlagStatus( USART2, USART_FLAG_TXE ) != SET );
	USART_ClearFlag( USART2 , USART_FLAG_TXE );
	USART_SendData( USART2 , Wchar ); 
}
void UART2_WriteBytes(unsigned char  *cmd_buf,unsigned char  offset,unsigned char n )
{
	int i=0,end;
	end=n+offset;
 for(i=offset;i<end;i++)
	{
	  while ( USART_GetFlagStatus( USART2, USART_FLAG_TXE ) != SET );
	  USART_ClearFlag( USART2 , USART_FLAG_TXE );
	  USART_SendData( USART2 , cmd_buf[i] ); 
	}
}

#define uart_buffer_size2   100
unsigned char uart_rec_buffer2[uart_buffer_size2];
unsigned char index_uart2=0;
unsigned char uart_buffer_head_ok2;
unsigned char cmd_in2=0;
unsigned char bq_cmd_type=0;
unsigned char max_board_num=0;
unsigned char board_addr_rd_ok=0;

extern unsigned char message[];





void USART2_IRQHandler(void)
{
	unsigned char  c,i;
 if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 	
	  c=USART2->DR;  
		if(c=='@') 
			{
				uart_buffer_head_ok2=1;
				index_uart2=0;
			}
	 	  if(uart_buffer_head_ok2==1)
		    {
		     uart_rec_buffer2[index_uart2++]=c;
				 if(c=='#') //end ok
				 {
					 cmd_in2=1;
				 }					
		    }else
				{
				 index_uart2=0;
				}
		  if(index_uart2>=uart_buffer_size2)  index_uart2=0;
		
	} 
	
	//UART1_WriteByte(c);
	
	USART_ClearITPendingBit( USART2 , USART_IT_RXNE );
}





 // CRC16??   
        int  wCRC16Table[] = {    
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,   
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,   
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,    
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,   
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,     
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,   
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,   
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,   
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,   
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,      
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,   
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,   
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,   
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,   
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,      
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,   
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,   
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,   
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,   
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,      
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,   
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,   
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,   
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,   
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,     
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,   
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,   
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,   
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,   
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,     
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,   
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};


        //////////////////////////////////////////////////////////////////////////   
        // ????: CRC16??   
        // ????: pDataIn: ????   
        //           iLenIn: ????              
        // ????: pCRCOut: 2?????    
        int CRC16(unsigned char *pDataIn,int pos, int iLenIn)
        {
            int wResult = 0;
            int wTableNo = 0;
            int i=0;
            for (  i = 0; i < iLenIn; i++)
            {
                wTableNo = ((wResult & 0xff) ^ (pDataIn[pos+i] & 0xff));
                wResult = ((wResult >> 8) & 0xff) ^ wCRC16Table[wTableNo];
            }

            return wResult;
        } 

 void UART3_RxInt( void )
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTy2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}
	void USART3_Init(u32 baud)   
{  
    USART_InitTypeDef USART_InitStructure;  
    NVIC_InitTypeDef NVIC_InitStructure;   
    GPIO_InitTypeDef GPIO_InitStructure;    //?????????,?????GPIO  
    //?????RCC??  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //??UART3??GPIOB???  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  

    //?????GPIO???  
    // Configure USART3 Rx (PB.11) as input floating    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);  

    // Configure USART3 Tx (PB.10) as alternate function push-pull  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_Init(GPIOB, &GPIO_InitStructure);  

    //????  
    USART_InitStructure.USART_BaudRate = baud;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  

USART_Init(USART3, &USART_InitStructure);						//函数实规则
	
	USART_Cmd( USART3, ENABLE );	//使能UART1
	
	USART_ClearFlag( USART3 , USART_FLAG_TXE|USART_FLAG_RXNE );
	
	USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
	 UART3_RxInt();
      
} 

void USART3_Sned_Char(u8 temp)        
{  
    USART_SendData(USART3,(u8)temp);      
    while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);  
      
}
void UART3_WriteByte( unsigned char Wchar )
{
	while ( USART_GetFlagStatus( USART3, USART_FLAG_TXE ) != SET );
	USART_ClearFlag( USART3 , USART_FLAG_TXE );
	USART_SendData( USART3 , Wchar ); 
}
void USART3_Sned_Char_Buff(u8 buf[],u32 len)  
{  
    u32 i;  
    for(i=0;i<len;i++)  
    USART3_Sned_Char(buf[i]);  
          
}		
void uart1_send_str(  char *str)
{
unsigned char i=0;
	while(str[i]!='\0')
	{
   UART1_WriteByte(str[i++]);
	}
}
void uart2_send_str(  char *str)
{
unsigned char i=0;
	while(str[i]!='\0')
	{
   UART2_WriteByte(str[i++]);
	}
}
void uart3_send_str(  char *str)
{
unsigned char i=0;
	while(str[i]!='\0')
	{
   UART3_WriteByte(str[i++]);
	}
}

#define uart_buffer_size3   100
unsigned char uart_rec_buffer3[uart_buffer_size3];
unsigned char index_uart3=0;
unsigned char uart_buffer_head_ok3;
unsigned char cmd_in3=0;
extern int init_state;
void USART3_IRQHandler(void)                    //??3??????
{
  unsigned char  c;
	 if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
	    c=USART3->DR;
		UART1_WriteByte(c);
	 
    
			 // 	UART1_WriteByte(c);
		// UART2_WriteByte(c);
		// USART3_Sned_Char(c);
		 // UART1_WriteByte(c); //eUART1_WriteByte('>'); UART1_WriteByte(c+1); 	  
	}
	
	USART_ClearITPendingBit( USART3 , USART_IT_RXNE );

}			
				