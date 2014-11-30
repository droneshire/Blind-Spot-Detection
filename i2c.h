/*
 * i2c.h
 *
 * Created: 11/29/2014 1:07:05 PM
 *  Author: Ross
 */ 

#ifndef I2C_H_
#define I2C_H_

#ifndef SOFT_SDA
#define SOFT_SDA PORTD3
#ifndef SOFT_SCL
#define SOFT_SCL PORTD4
#endif
#endif

#define I2C_DDR		DDRD
#define I2C_PORT	PORTD
#define I2C_PIN		PIND	

#define I2C_SDA_H()\
	I2C_DDR &= ~(1 << SOFT_SDA);\
	I2C_PORT |= (1 << SOFT_SDA); 
#define I2C_SDA_L()\
	I2C_DDR |= (1 << SOFT_SDA);\
	I2C_PORT &= ~(1 << SOFT_SDA); 
#define I2C_SCL_H()\
	I2C_DDR &= ~(1 << SOFT_SCL);\
	I2C_PORT |= (1 << SOFT_SCL);
#define I2C_SCL_L()\
	I2C_DDR |= (1 << SOFT_SCL);\
	I2C_PORT &= ~(1 << SOFT_SCL);

void i2cInit(void);
void i2cSetBitrate(unsigned short bitrateKHz);

void i2cWriteBit(unsigned char bit);
unsigned char i2cReadBit(void);
void i2cSendStart(void);
void i2cSendStop(void);

void i2cSendByte(unsigned char data);
unsigned char  i2cReceiveByte(unsigned char ackFlag);

#endif /* I2C_H_ */