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

//Delay used to generate clock
#define I2C_DELAY 2 //~250kHz clock
//brief Delay used for STOP condition
#define SCL_SDA_DELAY 1

#define I2C_SDA_H()\
	I2C_DDR &= ~(1 << SOFT_SDA);
#define I2C_SDA_L()\
	I2C_DDR |= (1 << SOFT_SDA);\
	I2C_PORT &= ~(1 << SOFT_SDA); 
#define I2C_SCL_H()\
	I2C_DDR &= ~(1 << SOFT_SCL);\
	while(!READ_SCL());
#define I2C_SCL_L()\
	I2C_DDR |= (1 << SOFT_SCL);\
	I2C_PORT &= ~(1 << SOFT_SCL);
	
#define READ_SCL() (I2C_PIN & (1 << SOFT_SCL))?1:0
#define READ_SDA() (I2C_PIN  & (1 << SOFT_SDA))?1:0
#define SET_SDA_OUT() I2C_DDR |= (1 << SOFT_SDA)


void SDA_delay(void);
	
void i2c_init(void);

void i2c_write_bit(unsigned char bit);
unsigned char i2c_read_bit(void);
void i2c_send_start(void);
void i2c_send_stop(void);

unsigned char i2c_send_byte(unsigned char data);
unsigned char  i2c_receive_byte(unsigned char ackFlag);

#endif /* I2C_H_ */