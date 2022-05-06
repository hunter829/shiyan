
#include "delay.h"
#include "software_iic.h"

void I2C_delay(void)

{

	u8 t = 2;

	while(t--);

	return;

}

int I2C_Start(void)

{

	I2C_SDA_OUT();

	

	SDA_H;

	SCL_H;

	I2C_delay();

	if(!SDA_read)

	{

		return DISABLE;

	}

	SDA_L;

	I2C_delay();

	if(SDA_read)

	{

		return DISABLE;

	}

	SCL_L;

	return ENABLE;

}

void I2C_Stop(void)

{

	I2C_SDA_OUT();	

	SCL_L;

	SDA_L;	

	SCL_H;

	I2C_delay();

	SDA_H;

	I2C_delay();

}

 

static void I2C_Ack()

{

	SCL_L;

	I2C_SDA_OUT();	

	

	SDA_L;

	I2C_delay();

	SCL_H;

	I2C_delay();

	SCL_L;

}

 

static void I2C_NoAck()

{

	SCL_L;

	I2C_SDA_OUT();

	

	I2C_delay();

	SDA_H;

	I2C_delay();

	SCL_H;

	I2C_delay();

	SCL_L;

}

uint8_t I2C_GetAck(void)

{

  uint8_t time = 0;

	I2C_SDA_IN();

	SDA_H;

	I2C_delay();

	SCL_H;

	I2C_delay();

	while(SDA_read)

	{

		time++;

		if(time > 250)

		{			

			SCL_L;

			return DISABLE;

		}

	}

	SCL_L;

	return ENABLE;

}

void I2C_SendByte(uint8_t Data)

{

  uint8_t cnt;

  I2C_SDA_OUT();	

  for(cnt=0; cnt<8; cnt++)

  {

    SCL_L;                              

    I2C_delay();

 

    if(Data & 0x80)

    {

      SDA_H;                         

    }

    else

    {

      SDA_L;                         

    }

    Data <<= 1;

    SCL_H;                              

    I2C_delay();

  }

  SCL_L;                                   

  I2C_delay();

}

 

 

uint8_t I2C_ReadByte(uint8_t ack)

{

  uint8_t cnt;

  uint16_t data;

  I2C_SDA_IN();	

	

  for(cnt=0; cnt<8; cnt++)

  {

    SCL_L;                                

    I2C_delay();

		

    SCL_H;                             

    data <<= 1;

    if(SDA_read)

    {

      data |= 0x01;                              

    }

    I2C_delay();

  }

  if(ack == 1)

  {

     I2C_NoAck();

  }

  else

  {

     I2C_Ack();

  }

  return data;                                  

}

void I2C_GPIO_Configuration(void)

{

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

 

  GPIO_InitStructure.GPIO_Pin = I2C_Pin_SCL;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

  GPIO_Init(ANO_GPIO_I2C,&GPIO_InitStructure);

	

  GPIO_InitStructure.GPIO_Pin = I2C_Pin_SDA;

  GPIO_Init(ANO_GPIO_I2C, &GPIO_InitStructure);

}

 

 

void I2C_SDA_IN()

{

   GPIO_InitTypeDef  GPIO_InitStructure;

   GPIO_InitStructure.GPIO_Pin = I2C_Pin_SDA;

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 

   GPIO_Init(ANO_GPIO_I2C, &GPIO_InitStructure);	

}

 

void I2C_SDA_OUT()

{

   GPIO_InitTypeDef  GPIO_InitStructure;

   GPIO_InitStructure.GPIO_Pin = I2C_Pin_SDA;

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 

   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	

   GPIO_Init(ANO_GPIO_I2C, &GPIO_InitStructure);	

}

void I2C_Initializes(void)

{

	I2C_GPIO_Configuration();

  SCL_H; 

	SDA_H;

}


int EEPROM_WriteByte(uint16_t Addr, uint8_t Data)

{

	I2C_Start();

	I2C_SendByte(EEPROM_DEV_ADDR | EEPROM_WR);

  if(!I2C_GetAck())

  {

	I2C_Stop();

	return DISABLE;

  }

  #if (8 == EEPROM_WORD_ADDR_SIZE)

  I2C_SendByte((uint8_t)(Addr&0x00FF));   

  #else

  I2C_SendByte((uint8_t)(Addr>>8));     

  I2C_SendByte((uint8_t)(Addr&0x00FF));

  #endif

  I2C_GetAck();

  I2C_SendByte(Data);

  I2C_Stop();

	return 1;

}

int EEPROM_ReadByte(uint16_t Addr, uint8_t *Data)

{

  I2C_Start(); 

	I2C_SendByte(EEPROM_DEV_ADDR | EEPROM_WR);

  if(!I2C_GetAck())

  {

	I2C_Stop();

	return DISABLE;

  }

  #if (8 == EEPROM_WORD_ADDR_SIZE)

  I2C_SendByte((uint8_t)(Addr&0x00FF)); 

  #else

  I2C_SendByte((uint8_t)(Addr>>8));   

	I2C_SendByte((uint8_t)(Addr&0x00FF));

  #endif

  I2C_Start();

	I2C_SendByte(EEPROM_DEV_ADDR | EEPROM_RD);

	

	if(!I2C_GetAck())

	{

	I2C_Stop();

	return DISABLE;

  }

  *Data = I2C_ReadByte(0); 

	I2C_Stop();

	return 1;

}

u8 Who_Am_I(void)

{

	u8 who;

	EEPROM_ReadByte(0x20, &who);

	return who;

}

 

//int Read_XYZ(void)

//{

// 	EEPROM_ReadByte(0x29, &x);

//	databuff[0] = x;

// 	EEPROM_ReadByte(0x2b, &y);

//	databuff[1] = y;

// 	EEPROM_ReadByte(0x2d, &z);

//	databuff[2] = z;

//	return ENABLE;

//}
