/*
 * i2c.h
 *
 * Created: 11/29/2014 1:07:05 PM
 *  Author: Ross
 */ 
#include "i2c.h"
#include "Arduino.h"
#include <util/delay.h>

static unsigned char started;

void SDA_delay() { volatile int v; int i; for (i=0; i < SCL_SDA_DELAY; i++) v; }
	
void i2c_init(void){
	//release the bus
	started = false;
	I2C_SCL_H();
	I2C_SDA_H();
}

void i2cDisable(void){
	I2C_DDR &= ~(1 << SOFT_SCL);
	I2C_DDR &= ~(1 << SOFT_SDA);
}

void i2cWriteBit(unsigned char bit){
	if(bit){
		I2C_SDA_H();
	}
	else{
		I2C_SDA_L();
	}
	I2C_SCL_H();
	_delay_us(I2C_DELAY);
	I2C_SCL_L();
	_delay_us(I2C_DELAY);
}

unsigned char i2cReadBit(void){
	unsigned char bit;
	I2C_SDA_H();
	I2C_SCL_H();
	bit = READ_SDA();
	_delay_us(I2C_DELAY);
	I2C_SCL_L();
	_delay_us(I2C_DELAY);
	return bit ? 1 : 0;
}

void i2c_send_start(void){
	if(started){
		I2C_SDA_H();
		_delay_us(I2C_DELAY);
		I2C_SCL_H();
		_delay_us(I2C_DELAY);
	}
	I2C_SDA_L();
	_delay_us(I2C_DELAY);
	I2C_SCL_L();
	_delay_us(I2C_DELAY);
	started = true;
}

void i2c_send_stop(void){
	I2C_SDA_L();
	_delay_us(I2C_DELAY);
	I2C_SCL_H();
	_delay_us(SCL_SDA_DELAY);
	I2C_SDA_H();
	started = false;
}

unsigned char i2c_send_byte(unsigned char data){
	unsigned char i;
	for(i = 0; i < 8; i++){
		i2cWriteBit((data & 0x80) != 0);
		data <<= 1;
	}
	I2C_SDA_H();
	I2C_SCL_H();
	if(READ_SDA())
		return 0;
		
	_delay_us(I2C_DELAY);
	I2C_SCL_L();
	SET_SDA_OUT();
	_delay_us(I2C_DELAY);
	return 1;
}

unsigned char i2c_receive_byte(unsigned char nack){
	unsigned char data, bit;
	data =0;
	for(bit = 0; bit < 8; bit++){
		data |= i2cReadBit();
		data <<= 1;
	}
	SET_SDA_OUT();
	if(nack){
		I2C_SDA_H();
		I2C_SCL_H();
		_delay_us(I2C_DELAY);
		I2C_SCL_L();
		_delay_us(I2C_DELAY);
	}
	else{
		I2C_SDA_L();
		I2C_SCL_H();
		_delay_us(I2C_DELAY);
		I2C_SCL_L();
		I2C_SDA_H();
		_delay_us(I2C_DELAY);
	}
	return data;
}
