/*
 * i2c.h
 *
 * Created: 11/29/2014 1:07:05 PM
 *  Author: Ross
 */ 
#include "i2c.h"
#include "Arduino.h"

static unsigned char i2c_delay;

void i2cInit(void){
	
}

void i2cSetBitrate(unsigned char bitrateKHz){
	if(bitrateKHz == 100)
		i2c_delay = 20;
	else
		i2c_delay = 5;
	
}

void i2cWriteBit(unsigned char bit){
	
}

unsigned char i2cReadBit(void){
	unsigned char bit;
	
	
	return bit ? 1 : 0;
}

void i2cSendStart(void){
	
}

void i2cSendStop(void){
	
}

void i2cWaitForComplete(void){
	
}

void i2cSendByte(unsigned char data){
	
}

unsigned char i2cReceiveByte(unsigned char ackFlag){
	
}

void delay_ms(unsigned short x)
{
	unsigned short y, z;
	for ( ; x > 0 ; x--){
		for ( y = 0 ; y < 90 ; y++){
			for ( z = 0 ; z < 6 ; z++){
				asm volatile ("nop");
			}
		}
	}
}
