/*
 * MMA8452REGS.h
 *
 * Created: 12/21/2013 1:30:55 PM
 *  Author: Ross Yeager
 *  Register set for the MMA8452 digital accelerometer
 */ 

#ifndef MMA8452REGS_H_
#define MMA8452REGS_H_

//Define the registers on the MMA8452

#define STATUS				0x00

#define OUT_X_MSB			0x01
#define OUT_X_LSB			0x02
#define OUT_Y_MSB			0x03
#define OUT_Y_LSB			0x04
#define OUT_Z_MSB			0x05
#define OUT_Z_LSB			0x06

#define SYSMOD				0x0B
#define INT_SOURCE			0x0C
#define WHO_AM_I			0x0D
#define XYZ_DATA_CFG		0x0E
#define HP_FILTER_CUTOFF	0x0F

#define PL_STATUS			0x10
#define PL_COUNT			0x12
#define PL_BF_ZCOMP			0x13
#define P_L_THS_REG			0x14

#define FF_MT_CFG			0x15
#define FF_MT_SRC			0x16
#define FF_MT_THS			0x17
#define FF_MT_COUNT			0x18

#define TRANSIENT_CFG		0x1D
#define TRANSIENT_SRC		0x1E
#define TRANSIENT_THS		0x1F
#define TRANSIENT_COUNT		0x20

#define PULSE_CFG			0x21
#define PULSE_SRC			0x22
#define PULSE_THSX			0x23
#define PULSE_THSY			0x24
#define PULSE_THSZ			0x25
#define PULSE_TMLT			0x26
#define PULSE_LTCY			0x27
#define PULSE_WIND			0x28

#define ASLP_COUNT			0x29

#define CTRL_REG1			0x2A
#define CTRL_REG2			0x2B
#define CTRL_REG3			0x2C
#define CTRL_REG4			0x2D
#define CTRL_REG5			0x2E

#define OFF_X				0X2F
#define OFF_Y				0X30
#define OFF_Z				0X31

//********************Bits**************


#define MODS				0
#define SMODS				3

//CTRL_REG1
#define ACTIVE				0
#define F_READ				1
#define LNOISE				2
#define DR0					3
#define DR1					4
#define DR2					5
#define ASLP_RATE0			6
#define ASLP_RATE1			7
//CTRL_REG2
#define MODS0				0
#define MODS1				1
#define SLPE				2
#define SMODS0				3
#define SMODS1				4
#define RST					6
#define ST					7
//CTRL_REG3
#define WAKE_TRANS 			6
#define WAKE_LNDPRT			5
#define WAKE_PULSE 			4
#define WAKE_FF_MT 			3
#define IPOL	 			1
#define PP_OD				0
//CTRL_REG4
#define INT_EN_ASLP			7
#define INT_EN_TRANS		5
#define INT_EN_LNDPRT		4
#define INT_EN_PULSE		3
#define INT_EN_FF_MT		2
#define INT_EN_DRDY			0
//CTRL_REG5
#define INT_CFG_ASLP		7
#define INT_CFG_TRANS		5
#define INT_CFG_LNDPRT		4
#define INT_CFG_PULSE		3
#define INT_CFG_FF_MT		2
#define INT_CFG_DRDY		0

//FF_MT_CFG
#define ELE					7
#define OAE					6
#define ZEFE				5
#define YEFE				4
#define XEFE				3
//

//********************BIT MASKS**************

#define bmRES800			0x00
#define bmRES400			0x04
#define bmRES200			0x10
#define bmRES100			0x14
#define bmRES50				0x20
#define bmRES12DOT5			0x24
#define bmRES6DOT25			0x30
#define bmRES1DOT56			0x34


#endif /* MMA8452REGS_H_ */