#ifndef MMA8452_H
#define MMA8452_H
#include "MMA8452REGS.h"

/************************************************************************/
//ACCELEROMETER CONSTANTS
/************************************************************************/
enum {STANDBY, WAKE, SLEEP};		//ACCELEROMETER STATES
enum {NORMAL, LPNP, HP, LP};		//POWER MODES
/************************************************************************/

class MMA8452
{
	private:
		unsigned char mAddress;
		
	public:
		MMA8452(unsigned char address);
		void MMA8452Active(void);
		void MMA8452Standby(void);
		void setBitrate(unsigned short bitrate);
		void writeRegister(unsigned char address, unsigned char data);
		unsigned char readRegister(unsigned char address);
		void readRegisters(unsigned char address, int i, unsigned char * dest);
		void readAccelData(int * destination);
		void initMMA8452(unsigned char fsr, unsigned char dr, unsigned char sr, unsigned char sc, unsigned char mt, unsigned char mdc);	
};
#endif
