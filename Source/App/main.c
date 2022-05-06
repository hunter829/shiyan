/*************************************
		杭州光子物联网技术有限公司
		https://gzwelink.taobao.com
				2015.08.01
**************************************/
#include "stm32f10x.h"
#include "ALL_Includes.h"
#include "uart.h"
#include "stdio.h"
#include "BitBand.h"
#include "software_iic.h"
#include "ADS1256.h"
#include "led.h"
extern unsigned char uart_rec_buffer[ ];
extern unsigned char index_uart ;
extern unsigned char uart_buffer_head_ok;
extern unsigned char cmd_in;

extern unsigned char uart_rec_buffer2[ ];
extern unsigned char index_uart2 ;
extern unsigned char uart_buffer_head_ok2;
extern unsigned char cmd_in2;

#define led1  	     PBOUT(5)
#define led2	       PEOUT(5)
#define sci_bus	     PAOUT(2)
#define pa3	         PAOUT(3)
#define beep PBOUT(8)

#define bus_busy    PAIN(2)
#define addr0       PBIN(9)
#define addr1       PAIN(3)
#define addr0_out   PBOUT(9)
#define addr1_out   PAOUT(3)

int  Ad_data[8];
float Volts[8];
#define  v_addar 0
#define  i_addar 1
unsigned char v_r_value=0;
unsigned char i_r_value=0;
unsigned char my_addr=0;
uint8_t n;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
vu32 ret; /* for return of the interrupt handling */
volatile TestStatus TestRx;
ErrorStatus HSEStartUpStatus;

TestStatus CAN_Polling(void);
TestStatus CAN_Interrupt(void);

void GPIO_Configuration(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
  	/* Configure CAN pin: RX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  	/* Configure CAN pin: TX */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  /* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	 /* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		/* Configure led 1 2  */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

//系统中断管理
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

  	/* Configure the NVIC Preemption Priority Bits */  
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	#ifdef  VECT_TAB_RAM  
	  /* Set the Vector Table base location at 0x20000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */ 
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif

	/* enabling interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

//配置系统时钟,使能各外设时钟
void RCC_Configuration(void)
{
	SystemInit();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
                           |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
                           |RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
						   |RCC_APB2Periph_ADC1  | RCC_APB2Periph_AFIO 
                           |RCC_APB2Periph_SPI1, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 
                           |RCC_APB1Periph_USART3|RCC_APB1Periph_TIM2	                           
                           , ENABLE );
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* CAN Periph clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

  char message[300];
//void uart1_send_str(  char *str)
//{
//unsigned char i=0;
//	while(str[i]!='\0')
//	{
//   UART1_WriteByte(str[i++]);
//	}
//}
 unsigned char hex3[2],hex2[2],hex1[2],dat[3];
//??? str[0]
void HexToStr(unsigned char val,unsigned char *str)
{
	//str[0]=_crol_(val,4) & 0xf;
	str[0]=(val& 0xf0)>>4;
	str[1]=val & 0xf;
	if (str[0]>9) str[0] +=55; else str[0] +=48;
	if (str[1]>9) str[1] +=55; else str[1] +=48;	
	//str[2]=0;
}//*/

//high in str[0];
unsigned char StrToHex(unsigned char val,unsigned char *str)
{
	unsigned int i=0;
	//str[0]=_crol_(val,4) & 0xf;
	if(str[0]>='0'&&str[0]<='9') str[0]=str[0]-'0';
	if(str[1]>='0'&&str[1]<='9') str[1]=str[1]-'0';
	if(str[0]>='a'&&str[0]<='f') str[0]=str[0]-'a'+10;
	if(str[1]>='a'&&str[1]<='f') str[1]=str[1]-'a'+10;
	if(str[0]>='A'&&str[0]<='F') str[0]=str[0]-'A'+10;
	if(str[1]>='A'&&str[1]<='F') str[1]=str[1]-'A'+10;
	i=str[0];
	i=(i<<4)+str[1];
	return (i);
}//*/

u8 CanTxData(void)
{
  CanTxMsg TxMessage;
  CanRxMsg RxMessage;
  u32 i = 0;
  u8 TransmitMailbox;
  /* transmit */
  TxMessage.StdId=0xf4;
  TxMessage.RTR=0x04;
  TxMessage.IDE=0x04;
	TxMessage.ExtId=0x1765432f;
	 
  TxMessage.DLC=8;
  TxMessage.Data[0]=9;
  TxMessage.Data[1]=2;
  TxMessage.Data[2]=3;
  TxMessage.Data[3]=4;
  TxMessage.Data[4]=5;
  TxMessage.Data[5]=6;
  TxMessage.Data[6]=7;
  TxMessage.Data[7]=1;

  TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
  i = 0;
  while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
  {
    i++;
  }

  i = 0;
  while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
  {
    i++;
  }

}

u8 Can_send_Data(unsigned int ext_frame_id,unsigned char *data,unsigned char data_n)
{
  CanTxMsg TxMessage;
  CanRxMsg RxMessage;
  u32 i = 0;
  u8 TransmitMailbox;
  /* transmit */
  TxMessage.StdId=0xf4;
  TxMessage.RTR=0x04;
  TxMessage.IDE=0x04;
	TxMessage.ExtId=ext_frame_id;
	 
  TxMessage.DLC=data_n;
	for(  i=0;i<data_n;i++)
     TxMessage.Data[i]=data[i];
  

  TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
  i = 0;
  while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
  {
    i++;
  }

  i = 0;
  while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
  {
    i++;
  }
 ///////////////  CAN ?? ??
			 if(CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF)==SET)
        {
                                // CAN_Config();
                        //        CAN_ClearFlag(CANx, CAN_FLAG_BOF);
           CAN1->MCR|=1;
           CAN1->MCR&=0xfffffffe;

           sprintf(message,"get can1 off!resend!\r\n"    );
			      uart1_send_str(message);
					 TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
           i = 0;
           while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (i != 0xFF))
           {
            i++;
           }

            i = 0;
           while((CAN_MessagePending(CAN1,CAN_FIFO0) < 1) && (i != 0xFF))
           {
            i++;
           }
					
					
          } // end of can err judge
}

unsigned char cmp_data(  char *data1,unsigned char *data2,unsigned char n)
{
unsigned char i=0;
	for(i=0;i<n;i++)
	{
	 if(data1[i]!=data2[i]) return i;
	}
	return i;
}

//配置所有外设
void Init_All_Periph(void)
{
	Delay_Init(72);
	RCC_Configuration();
	//LcdGpioConfig();
	//Init_lcd();	
	GPIO_Configuration();
	NVIC_Configuration();
}
CanRxMsg RxMessage;


unsigned char       CRM_data[]={0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char  TPCM_RTS_data[]={0x10,0x29,0x00,0x06,0xff,0x00,0x02,0x00};
unsigned char  TPCM_CTS_data[]={0x11,0x06,0x01,0xff,0xff,0x00,0x02,0x00};
unsigned char  TPCM_DT_data1[]= {0x01,0x00,0x01,0x00,0x04,0x8c,0x0a,0xf8};
unsigned char  TPCM_DT_data2[]= {0x02,0x15,0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char  TPCM_DT_data3[]= {0x03,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char  TPCM_DT_data4[]= {0x04,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char  TPCM_DT_data5[]= {0x05,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char  TPCM_DT_data6[]= {0x06,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
unsigned char      CRM2_data[]= {0xaa,0x01,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char  TPCM_RTS2_data[]={0x10,0x0d,0x00,0x02,0xff,0x00,0x06,0x00};
unsigned char  TPCM_CTS2_data[]={0x11,0x02,0x01,0xff,0xff,0x00,0x06,0x00};
unsigned char  TPCM_DT2_data1[]= {0x01,0xa2,0x01,0x3f,0x0f,0x8c,0x0a,0xf4};
unsigned char  TPCM_DT2_data2[]= {0x02,0x08,0x66,0xf4,0x01,0x34,0x08,0xff};
unsigned char         TPCM_EM[]= {0x13,0x0d,0x00,0x02,0xff,0x00,0x06,0x00};
unsigned char             CCS[]= {0x00,0x00,0xa0,0x0f,0x00,0x00,0x00,0x00};
unsigned char             BSM[]= {0x40,0x42,0x02,0x40,0x02,0x00,0x10,0x00};

                                 //low high 
unsigned char     my_bcl_data[]= {0x34,0x08,0x5a,0x0f,0x01,0x00,0x00,0x00};
//  0834  210v

void   can_dat_uart1()
{
int i=0;
 if(RxMessage.DLC)//接收长度有效
		 {
			  
			 
			 
			 if(RxMessage.ExtId==0x11390000)
			 {
			   TestRx=Can_send_Data(0x12800000+my_addr,my_bcl_data,8);   
				 RxMessage.DLC=0;
				 delay_us(50);
			 
			 }
			 sprintf(message,"@%d,CAN,n=%d:,ID=%8x,data%2x%2x%2x%2x%2x%2x%2x%2x#\r\n" ,my_addr,RxMessage.DLC,RxMessage.ExtId,RxMessage.Data [0],RxMessage.Data [1],RxMessage.Data [2],RxMessage.Data [3],RxMessage.Data [4],RxMessage.Data [5],RxMessage.Data [6],RxMessage.Data [7]  );
			 for(n=0;n<50;n++)                    ///  空格换 0
			 
			 {
				 if(*(message+n)==' ')*(message+n)='0';
				 if(*(message+n)=='#')break;
			 }
			 
			
			 
			 uart1_send_str(message);
			 
			 ///////////////////////////////////
			 RxMessage.DLC=0;
		 }
		 // release bus
		 delay_us(200);
		 sci_bus=1;
}


unsigned int get_can_id_from_uart(void)
{
	int i=0;
 hex1[0]=uart_rec_buffer[14];
 hex1[1]=uart_rec_buffer[15];	
 i=StrToHex(0,hex1);
 hex1[0]=uart_rec_buffer[16];
 hex1[1]=uart_rec_buffer[17];	
 i=(i<<8)+StrToHex(0,hex1);
 hex1[0]=uart_rec_buffer[18];
 hex1[1]=uart_rec_buffer[19];	
 i=(i<<8)+StrToHex(0,hex1);
 hex1[0]=uart_rec_buffer[20];
 hex1[1]=uart_rec_buffer[21];	
 i=(i<<8)+StrToHex(0,hex1);	
	
	return(i);
}
unsigned char  get_can_data(unsigned char *str)
{
 unsigned char n=0,i=0;
	n=uart_rec_buffer[9+2]-'0';
	if(n>=8) n=8;
	for(i=0;i<n;i++)
	{
	 hex1[0]=uart_rec_buffer[23+2*i];
   hex1[1]=uart_rec_buffer[23+2*i+1];	
   str[i]=StrToHex(0,hex1);
	}
	
return(n);
}

#define myaddr 0


/*******************************************************************************
* Function Name  : CAN_Polling
* Description    : Configures the CAN and transmit and receive by polling
* Input          : None
* Output         : None
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
TestStatus CAN_Polling(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CanRxMsg RxMessage;
  u8 TransmitMailbox;

  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler=9;
  CAN_Init(CAN1,&CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=0;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);



  /* receive */
  RxMessage.StdId=0x00;
  RxMessage.IDE=CAN_ID_STD;
  RxMessage.DLC=0;
  RxMessage.Data[0]=0x00;
  RxMessage.Data[1]=0x00;
  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

  if (RxMessage.StdId!=0x11) return FAILED;

  if (RxMessage.IDE!=CAN_ID_STD) return FAILED;

  if (RxMessage.DLC!=2) return FAILED;

  if ((RxMessage.Data[0]<<8|RxMessage.Data[1])!=0xCAFE) return FAILED;
  
  return PASSED; /* Test Passed */
}

/*******************************************************************************
* Function Name  : CAN_Interrupt
* Description    : Configures the CAN and transmit and receive by interruption
* Input          : None
* Output         : None
* Return         : PASSED if the reception is well done, FAILED in other case
*******************************************************************************/
TestStatus CAN_Interrupt(void)
{
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CanTxMsg TxMessage;
  u32 i = 0;

  /* CAN register init */
  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM=DISABLE;
  CAN_InitStructure.CAN_ABOM=DISABLE;
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=DISABLE;
  CAN_InitStructure.CAN_RFLM=DISABLE;
  CAN_InitStructure.CAN_TXFP=DISABLE;
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
  CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler=9;
  CAN_Init(CAN1,&CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber=1;
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);

  /* CAN FIFO0 message pending interrupt enable */ 
  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);


  return (TestStatus)0;
}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{

  CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);

}

#define myaddr 0
#include "stmflash.h"
 struct para_type *my_para;
 struct para_type my_para_real;
  struct SOC_type *my_soc;
 struct SOC_type my_soc_real;
int Adc;
signed char cnt =0;
void read_ads1256()
{
	int i=0;
	
	ADS1256ReadData( (i << 4) | ADS1256_MUXN_AINCOM);
for(i = 1;i <= 8;i++) 
	{
         Adc = ADS1256ReadData( (i << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		
         //delay_ms(50);
			   /*差分采集方式*/
		     //	Adc = ADS1256ReadData( ADS1256_MUXP_AIN0|ADS1256_MUXN_AIN1); //P = AIN0 ,N = AIN1 差分方式*/
			   cnt =i ; 
			   Volts[cnt] = Adc*0.000000598;
			   //if(	Volts[cnt]<0)Volts[cnt]+=5;
			   Ad_data[cnt]= Adc;
         			 
			 }
}
void process_cmd1()
{
	unsigned char cmp_len,i,check,cmd_buf[30];
	unsigned	int plc_addr,plc_val,temp_int1,temp_int2;
	u8 ch_num,set_num;
	  
	 sprintf(message,"@0,setid,"   );   
	   cmp_len=strlen(message)-1;
		 if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			{
			//		ch_num = uart_rec_buffer[cmp_len-2]-'0';
				cmp_len=cmp_len+1;
				temp_int1=uart_rec_buffer[cmp_len]-'0';
				for(i=1;i<10;i++)
				{
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_int1=temp_int1*10+uart_rec_buffer[cmp_len+i]-'0';					
				}
				
				sprintf(message," @%d,getid,%d,# \r\n",my_addr,temp_int1  );
        uart1_send_str(message);
				my_addr=temp_int1;
				delay_ms(100);
				
				sprintf(message," @0,setid,%d,# \r\n",my_addr+1  );
        uart2_send_str(message);
				delay_ms(10);
				return;
			}
			
	
	sprintf(message,"@%d," , my_addr );   
	  cmp_len=strlen(message)-1;
	if( cmp_data(message,uart_rec_buffer2,cmp_len)!=cmp_len)	
	  {			
			for( i=0;i<100;i++)
			{
			  UART2_WriteByte(uart_rec_buffer[i]);
				if(uart_rec_buffer[i]=='#')
					break;				
			}
				
			//return;
	  }
	
 
	
	
		sprintf(message,"@%d,readads1256," , my_addr );   
	  cmp_len=strlen(message)-1;
		if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			{
				cmp_len=cmp_len+1;
				temp_int1=uart_rec_buffer[cmp_len]-'0';
				if(temp_int1==9)
				{
				  Adc = ADS1256ReadData( (i << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		
            delay_ms(50);
			   
				 for(i = 1;i <=8;i++) 
	        {
             Adc = ADS1256ReadData( (i << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		
            delay_ms(50);
			   //  Adc = ADS1256ReadData( (i << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		
           

						/*差分采集方式*/
		     //	Adc = ADS1256ReadData( ADS1256_MUXP_AIN0|ADS1256_MUXN_AIN1); //P = AIN0 ,N = AIN1 差分方式*/
			   cnt =i ;
			    
			   Volts[cnt] = Adc*0.000000598;
			   //if(	Volts[cnt]<0)Volts[cnt]+=5;
			   Ad_data[cnt]= Adc;
         sprintf(message,"@%d,Adc Data ch%d :0x%8x ,VOL  ,%.5fV,# \r\n",my_addr,cnt,Ad_data[cnt],Volts[cnt] );
          uart1_send_str(message);
					
			   }
			 }
			if(temp_int1>=0&&temp_int1<=7)
				{
				 Adc = ADS1256ReadData( (temp_int1 << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		
            delay_ms(50);
					 Adc = ADS1256ReadData( (temp_int1 << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		
            delay_ms(50);
					 Volts[temp_int1] = Adc*0.000000598;
			   //if(	Volts[cnt]<0)Volts[cnt]+=5;
			   Ad_data[temp_int1]= Adc;
         sprintf(message,"@%d,Adc Data ch%d :0x%8x ,VOL  ,%.5fV,# \r\n",my_addr,temp_int1,Ad_data[temp_int1],Volts[temp_int1] );
          uart1_send_str(message);
				}
					
					
			 
			}
			sprintf(message,"@%d,setad5243," , my_addr );   
	  cmp_len=strlen(message)-1;
		if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			{
				cmp_len=cmp_len+1;
				temp_int1=uart_rec_buffer[cmp_len]-'0';
				for(i=1;i<10;i++)
				{
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_int1=temp_int1*10+uart_rec_buffer[cmp_len+i]-'0';					
				}
				//EEPROM_WriteByte(i_addar,temp_int1);
				cmp_len=cmp_len+1+i;
				temp_int2=uart_rec_buffer[cmp_len]-'0';
				for(i=1;i<10;i++)
				{
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_int2=temp_int2*10+uart_rec_buffer[cmp_len+i]-'0';					
				}
				EEPROM_WriteByte(0,temp_int1 );delay_ms(100);
				EEPROM_WriteByte(0x80,temp_int2 ); 
				
				sprintf(message,"@%d,Ad5243 set 1= %d,2= %d,# \r\n",my_addr,temp_int1,temp_int2 );
        uart1_send_str(message);
				
			}
			
		sprintf(message,"@%d,setswitch," , my_addr );   
	  cmp_len=strlen(message)-1;
		if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			{
				cmp_len=cmp_len+1;
				temp_int1=uart_rec_buffer[cmp_len]-'0';
				for(i=1;i<10;i++)
				{
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_int1=temp_int1*10+uart_rec_buffer[cmp_len+i]-'0';					
				}
				//EEPROM_WriteByte(i_addar,temp_int1);
				cmp_len=cmp_len+1+i;
				temp_int2=uart_rec_buffer[cmp_len]-'0';
				for(i=1;i<10;i++)
				{
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_int2=temp_int2*10+uart_rec_buffer[cmp_len+i]-'0';					
				}
			 switch(temp_int1)
			 {
				 case 1:light_ctrl_switch1=temp_int2;break;
			   case 2:light_ctrl_switch2=temp_int2;break;
				 case 3:light_ctrl_switch3=temp_int2;break;
			   case 4:light_ctrl_switch4=temp_int2;break;
				 case 5:light_ctrl_switch5=temp_int2;break;
			   case 6:light_ctrl_switch6=temp_int2;break;
			 }
				
				sprintf(message,"@%d,setswitch,%d,%d,# \r\n",my_addr,temp_int1,temp_int2 );
        uart1_send_str(message);
				
			}
			sprintf(message,"@%d,all_light," , my_addr );   
	  cmp_len=strlen(message)-1;
		if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
			{
				cmp_len=cmp_len+1;
				temp_int1=uart_rec_buffer[cmp_len]-'0';
				for(i=1;i<10;i++)
				{
					if(uart_rec_buffer[cmp_len+i]==',') break;
				  temp_int1=temp_int1*10+uart_rec_buffer[cmp_len+i]-'0';					
				}
				 
			  light_ctrl_switch1=temp_int1; 
			  light_ctrl_switch2=temp_int1; 
				light_ctrl_switch3=temp_int1; 
			  light_ctrl_switch4=temp_int1; 
				light_ctrl_switch5=temp_int1; 
			  light_ctrl_switch6=temp_int1; 
				sprintf(message,"@%d,all_light,%d ,# \r\n",my_addr,temp_int1 );
        uart1_send_str(message);
				
			 }
				
				
			 
			
			
	//////////end of rs232 cmd send/////////////////
			
}



void process_cmd2()
{
	unsigned char cmp_len,i,check,cmd_buf[30];
	unsigned	int plc_addr,plc_val,temp_int1,temp_int2;
	u8 ch_num,set_num;
	////////// rs232 cmd send  testplcy0 /////////////////
	sprintf(message,"@%d," , my_addr );   
	  cmp_len=strlen(message)-1;
	if( cmp_data(message,uart_rec_buffer2,cmp_len)!=cmp_len)	
	  {			
			for( i=0;i<100;i++)
			{
			  UART1_WriteByte(uart_rec_buffer2[i]);
				if(uart_rec_buffer2[i]=='#')
					break;				
			}
			  sprintf(message,"\r\n"   );
 			      uart1_send_str(message);
			return;
	  }
	
//	
//	  sprintf(message,"@%d," , my_addr );   
//	   cmp_len=strlen(message)-1;
//		if( cmp_data(message,uart_rec_buffer,cmp_len)==cmp_len)		
//			{
//				cmd_buf[0]=':';
//				cmd_buf[0]=0x01;
//				cmd_buf[0]=0x05;

//			}
}


int main(void)
{  
	unsigned char i,data[10],can_data_n_send,can_tx_data[10];
	unsigned int data_id,main_cnt;
	unsigned char bms_step=0;
	float max_chager_voletage=0;
	float min_chager_voletage=0;
	float max_chager_current=0;
	unsigned char ccs_cnt=0;
	int para_set=0;
	int scan_ads_1256_index=0;
	int v_cnt=0;
	Init_All_Periph();
	UART1_Config(115200);
  UART2_Config(115200);
 	I2C_Initializes();
 Init_ADS1256_GPIO(); //初始化ADS1256 GPIO管脚 
	delay_ms(500);
	 ADS1256_Init();

	LED_GPIO_Config();
	light_ctrl_switch1=1;
	light_ctrl_switch2=1;
	light_ctrl_switch3=1;
	light_ctrl_switch4=1;
	light_ctrl_switch5=1;
	light_ctrl_switch6=1;
	
	  __disable_irq() ;//NVIC_SETPRIMASK();         //?????					 
	  para_setting();
    __enable_irq();
	 	delay_ms(10 ); 
	my_addr=0; 
//sprintf(message,"@%d,reset\r\n#" ,my_addr  );
//			 uart1_send_str(message); 
//	delay_ms(2000); 
// sprintf(message,"@%d,CAN,n=%d:,ID=%8x,%d\r\n#" ,my_addr,RxMessage.DLC,RxMessage.ExtId,bus_busy );
//			 uart1_send_str(message);
 
 
 
 delay_ms(200);
	 
	
	cmd_in2=0;
	index_uart2=0;
 cmd_in=0;
	index_uart=0;
	
 	while(1)
  	{
			beep=0;
			led1=!led1;
			led2=!led2;
		  main_cnt++; 
			delay_us(10); 
			read_ads1256();
			
			
					sprintf(message,"channel1 volt=%f V\r\n",Volts[1]) ;
					uart1_send_str(message);
					delay_ms(1000);
				
	    //for(i = 0;i < 8;i++)
//      if(main_cnt%50==0)		  
//			{
//         Adc = ADS1256ReadData( (scan_ads_1256_index << 4) | ADS1256_MUXN_AINCOM);// 相当于 ( ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);		
//         //delay_ms(50);
//			   /*差分采集方式*/
//		     //	Adc = ADS1256ReadData( ADS1256_MUXP_AIN0|ADS1256_MUXN_AIN1); //P = AIN0 ,N = AIN1 差分方式*/
//			   cnt =scan_ads_1256_index-1;
//			   if(cnt<0)
//				   cnt+=8;
//			   Volts[cnt] = Adc*0.000000598;
//			   //if(	Volts[cnt]<0)Volts[cnt]+=5;
//			   Ad_data[cnt]= Adc;
//         scan_ads_1256_index++;	
//         if(scan_ads_1256_index>=8) scan_ads_1256_index=0;				 
//			 }
			 
			 
//			 if(cmd_in==1)
//			 {
//				 process_cmd1();
//				  
//					cmd_in=0;
//			 }// end of cmd in
//			
//			  if(cmd_in2==1)
//			 {
//				  process_cmd2();
				 
				 
//				 for( i=0;i<100;i++)
//			   {
//			      UART1_WriteByte(uart_rec_buffer2[i]);
//				    if(uart_rec_buffer2[i]=='#')
//					      break;				
//			   }
//				  sprintf(message,"\r\n" ,my_addr  );
//			      uart1_send_str(message);
//				   sprintf(message,"@%d,get cmd2 #\r\n" ,my_addr  );
//			      uart1_send_str(message); 
//					cmd_in2=0;
			// } end of cmd in
			  
			 
			  
  	// }end of main while
		
	
}


}