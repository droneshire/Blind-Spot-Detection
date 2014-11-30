/*
 * i2c.h
 *
 * Created: 11/29/2014 1:07:05 PM
 *  Author: Ross
 */ 
#include "i2c.h"
#include "Arduino.h"

static unsigned char I2C_DELAY_US;
static unsigned char started;

void i2cInit(void){
	//release the bus
	started = false;
	I2C_DDR &= ~((1 << SOFT_SDA) | (1 << SOFT_SCL)); 
}

void i2cSetBitrate(unsigned short bitrateKHz){
	if(bitrateKHz == 100)
		I2C_DELAY_US = 4;
	else
		I2C_DELAY_US = 1;
}

void i2cWriteBit(unsigned char bit){
	unsigned short timeout;
	timeout = 0;
	if(bit){
		I2C_SDA_H();
	}
	else{
		I2C_SDA_L();
	}
	delayMicroseconds(I2C_DELAY_US);
	I2C_SCL_H();
	while((I2C_PORT & (1 << SOFT_SCL)) == 0){
		if(timeout++ > 90)
			break;
	}
	delayMicroseconds(I2C_DELAY_US);
	I2C_SCL_L();
}

unsigned char i2cReadBit(void){
	unsigned char bit;
	unsigned short timeout;
	timeout = 0;
	I2C_SDA_H();
	delayMicroseconds(I2C_DELAY_US);
	I2C_SCL_H();
	while((I2C_PORT & (1 << SOFT_SCL)) == 0){
		if(timeout++ > 90)
			break;
	}
	bit = I2C_PORT & (1 << SOFT_SDA);
	delayMicroseconds(I2C_DELAY_US);
	I2C_SCL_L();
	return bit ? 1 : 0;
}

void i2cSendStart(void){
	unsigned short timeout;
	timeout = 0;
	if(started){
		I2C_SDA_H();
		delayMicroseconds(I2C_DELAY_US);
		I2C_SCL_H();
		while((I2C_PORT & (1 << SOFT_SCL)) == 0){
			if(timeout++ > 90)
				break;
		}
		delayMicroseconds(I2C_DELAY_US);
	}
	I2C_SDA_L();
	delayMicroseconds(I2C_DELAY_US);
	I2C_SCL_L();
	started = true;
}

void i2cSendStop(void){
	unsigned short timeout;
	timeout = 0;
	I2C_SDA_L();
	delayMicroseconds(I2C_DELAY_US);
	I2C_SCL_H();
	while((I2C_PORT & (1 << SOFT_SCL)) == 0){
		if(timeout++ > 90)
			break;
	}
	delayMicroseconds(I2C_DELAY_US);
	I2C_SDA_H();
	delayMicroseconds(I2C_DELAY_US);
	started = false;
}

unsigned char i2cWaitForAck(void){
	unsigned short timeout;
	timeout = 0;
	I2C_SDA_H(); //release the SDA line
	delayMicroseconds(I2C_DELAY_US);
	I2C_SCL_H();
	while(!(I2C_PORT & (1 << SOFT_SCL))){
		if(timeout++ > 90)
			break;
	}
	timeout = 0;
	while((I2C_PORT & (1 << SOFT_SDA))){
		if(timeout++ > 90)
			return 0;	//did not receive ack
	}
	return 1;	//received an ack
}


unsigned char i2cSendByte(unsigned char data){
	unsigned char i;
	for(i = 0; i < 8; i++){
		i2cWriteBit(data & 0x80);
		data <<= 1;
	}
	//return i2cWaitForAck();
	return 1;
}

unsigned char i2cReceiveByte(unsigned char ackFlag){
	unsigned char data, i;
	data =0;
	for(i = 0; i < 8; i++){
		data |= i2cReadBit();
		data <<= 1;
	}
	
	i2cWriteBit(ackFlag ? 0 : 1);
	delayMicroseconds(I2C_DELAY_US);
	return data;
}
