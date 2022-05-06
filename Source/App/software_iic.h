
#include "stm32f10x.h"

#define ANO_GPIO_I2C    GPIOC

#define I2C_Pin_SCL     GPIO_Pin_14

#define I2C_Pin_SDA     GPIO_Pin_13

#define ANO_RCC_I2C     RCC_APB2Periph_GPIOC

/*********************************************/

 

#define SCL_H         ANO_GPIO_I2C->BSRR = I2C_Pin_SCL

#define SCL_L         ANO_GPIO_I2C->BRR  = I2C_Pin_SCL

#define SDA_H         ANO_GPIO_I2C->BSRR = I2C_Pin_SDA

#define SDA_L         ANO_GPIO_I2C->BRR  = I2C_Pin_SDA

#define SCL_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL

#define SDA_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA

 

#define EEPROM_DEV_ADDR           0x5e          //??(????)

#define EEPROM_WR                 0x00                     //?

#define EEPROM_RD                 0x01                     //?

#define EEPROM_WORD_ADDR_SIZE      8

//extern u8 databuff[3];

//extern u8 x,y,z;

int I2C_Start(void);

void I2C_Stop(void);

void I2C_Ack(void);

void I2C_NoAck(void);

void I2C_SDA_OUT(void);

void I2C_SDA_IN(void);

uint8_t I2C_GetAck(void);

void I2C_SendByte(uint8_t Data);

uint8_t I2C_ReadByte(uint8_t ack);

void I2C_delay(void);

int EEPROM_ReadByte(uint16_t Addr, uint8_t *Data);

int EEPROM_WriteByte(uint16_t Addr, uint8_t Data );

void I2C_Initializes(void);

u8 Sanzhou_Start(void);

u8 Who_Am_I(void);

int Read_XYZ(void);

 
