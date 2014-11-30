#include "MMA8452.h"
#include "i2c.h"
#include "Arduino.h"

MMA8452::MMA8452(unsigned char address)
{
	mAddress = address;
	i2cInit();
}

void MMA8452::MMA8452Active(void)
{	
	unsigned char c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c | 0x01);
}

void MMA8452::MMA8452Standby(void)
{
	unsigned char c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c & ~(0x01));
}

void MMA8452::setBitrate(unsigned short bitrate)
{
	i2cSetBitrate(bitrate);
}

void MMA8452::writeRegister(unsigned char address, unsigned char data)
{
  i2cSendStart();
  //i2cWaitForComplete();

  i2cSendByte((mAddress<<1)); // Write 0xB4
  //i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  //i2cWaitForComplete();

  i2cSendByte(data);
  //i2cWaitForComplete();

  i2cSendStop();
}

unsigned char MMA8452::readRegister(uint8_t address)
{
  unsigned char data;

  i2cSendStart();
  //i2cWaitForComplete();

  i2cSendByte((mAddress<<1)); // Write 0x3C
  //i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  //i2cWaitForComplete();

  i2cSendStart();

  i2cSendByte((mAddress<<1)|0x01); // Write 0x3D
  //i2cWaitForComplete();
  //i2cReceiveByte(true);
  //i2cWaitForComplete();

  data = i2cReceiveByte(true);	// Get MSB result
  //i2cWaitForComplete();
  //i2cSendStop();

  //cbi(TWCR, TWEN);	// Disable TWI
  //sbi(TWCR, TWEN);	// Enable TWI

  return data;
}

void MMA8452::readRegisters(unsigned char address, int i, unsigned char * dest)
{
  i2cSendStart();
  //i2cWaitForComplete();

  i2cSendByte((mAddress<<1)); // write 0xB4
  //i2cWaitForComplete();

  i2cSendByte(address);	// write register address
  //i2cWaitForComplete();

  i2cSendStart();
  i2cSendByte((mAddress<<1)|0x01); // write 0xB5
  //i2cWaitForComplete();
  for (int j=0; j<i; j++)
  {
    //i2cReceiveByte(true);
    //i2cWaitForComplete();
    //dest[j] = i2cGetReceivedByte(); // Get MSB result
	dest[j] = i2cReceiveByte(true);
  }
  //i2cWaitForComplete();
  i2cSendStop();

  //cbi(TWCR, TWEN); // Disable TWI
  //sbi(TWCR, TWEN); // Enable TWI
}

void MMA8452::readAccelData(int * destination)
{
  unsigned char rawData[6];  // x/y/z accel register data stored here

  readRegisters(OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array

  //Loop to calculate 12-bit ADC and g value for each axis
  for (int i=0; i<6; i+=2)
  {
    destination[i/2] = ((rawData[i] << 8) | rawData[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
    if (rawData[i] > 0x7F)
    {  
      //If the number is negative, we have to make it so manually (no 12-bit data type)
      destination[i/2] = ~destination[i/2] + 1;
      destination[i/2] *= -1;  // Transform into negative 2's complement #
    }
  }
}

void MMA8452::initMMA8452(unsigned char fsr, unsigned char dr, unsigned char sr, unsigned char sc, unsigned char mt, unsigned char mdc)
{
	MMA8452Standby();
	//Set up the full scale range to 2, 4, or 8g.
	if ((fsr==2)||(fsr==4)||(fsr==8))
		writeRegister(XYZ_DATA_CFG, fsr >> 2);  
	else
		writeRegister(XYZ_DATA_CFG, 0);
    
	writeRegister(CTRL_REG1, readRegister(CTRL_REG1) & ~(0xF8));
	if (dr<= 7)
		writeRegister(CTRL_REG1, readRegister(CTRL_REG1) | (dr << DR0));
	if (sr<=3)
		writeRegister(CTRL_REG1, readRegister(CTRL_REG1) | (sr << ASLP_RATE0));
	
	writeRegister(CTRL_REG2, readRegister(CTRL_REG2) | (LP << SMODS) | (LP << MODS) | (1 << SLPE));  //LOW POWER MODE IN SLEEP & ACTIVE STATES WITH LOWEST SAMPLING
	writeRegister(ASLP_COUNT, sc);
	
	// Set up interrupt 1 and 2: 1 = wake ups, 2 = data
	writeRegister(CTRL_REG3, (1 << WAKE_FF_MT) | (1 << IPOL));  // Active high, push-pull interrupts, sleep wake up from motion detection
	writeRegister(CTRL_REG4, (1 << INT_EN_ASLP) | (1 << INT_EN_FF_MT) |  (1 << INT_EN_DRDY));  // DRDY ENABLE SLP/AWAKE INTERRUPT (INTERRUPT THROWN WHENEVER IT CHANGES) & MOTION INTERRUPT TO KEEP AWAKE
	//writeRegister(CTRL_REG5, (1 << INT_CFG_ASLP) | (1 << INT_CFG_FF_MT));  // DRDY on INT1, ASLP_WAKE INT2, FF INT2
	writeRegister(CTRL_REG5, (1 << INT_CFG_ASLP) | (1 << INT_CFG_FF_MT));  // DRDY on INT2, ASLP_WAKE INT1, FF INT1
	writeRegister(CTRL_REG5, readRegister(CTRL_REG5));
		
    //SETUP THE MOTION DETECTION
    writeRegister(FF_MT_CFG, 0xF8);  	/*MOTION DETECTION AND LATCH THE 
										//RESULT WHEN IT HAPPENS AS OPPOSED 
										//TO COMBINATIONAL REAL TIME*/
    writeRegister(FF_MT_THS, mt);		//MOTION DETECTION THRESHOLDS 
    writeRegister(FF_MT_COUNT, mdc);  	//TIME MOTION NEEDS TO BE 
										//PRESENT ABOVE THE THRESHOLD BEFORE INTERRUPT CAN BE ASSERTED
	
    MMA8452Active();
}

