//#START_MODULE_HEADER////////////////////////////////////////////////////////
//#
//# Filename:		blind_spot_detector.c
//#
//# Description:	This program is the main program for the blind spot detector
//#
//#								I/O's
//#				        	  __________
//#				INT_ACC1-----|			|-----SDA
//#				INT_ACC2-----|	 MCU	|-----SCL
//#				 TRIGGER-----|			|-----CE_REG
//#					ECHO-----|			|-----LED_CTRL
//#					CE_REG1--|			|-----MISO
//#				BATT_READ----|			|-----MOSI
//#					RESET----|			|-----SCK
//#				VCC_SOLAR----|			|-----BR_CTRL
//#					         |__________|
//#
//#
//# Authors:     	Ross Yeager
//#
//#END_MODULE_HEADER//////////////////////////////////////////////////////////

#include "Arduino.h"
#include "MMA8452REGS.h"
#include "NewPing.h"
#include "MMA8452.h"
#include <avr/sleep.h>

//#include "i2c.h"
#define RDV_DEBUG

//PIN DEFINITIONS

//blind spot detector pins
#define CE_REG		PORTB1	//OUTPUT
#define CE_REG1		PORTB2	//OUTPUT
#define INT1_ACC	PORTB4	//INPUT INTERRUPT
#define VCC_SOLAR	PORTC0	//INPUT ADC 
#define BATT_READ	PORTC1	//INPUT ADC 
#define BR_CNTL		PORTC2	//OUTPUT
#define INT2_ACC	PORTC3	//INPUT INTERRUPT
#define LED_CNTL	PORTD2	//OUTPUT PWM
#define ECHO_3V3	PORTD5	//INPUT
#define TRIGGER		PORTD6	//OUTPUT

//BIT-BANG I2C PINS
#define SOFT_SCL	PORTD4	//INOUT
#define SOFT_SDA	PORTD3	//INOUT

/************************************************************************/
//ACCELEROMETER CONSTANTS: these all need to be translated to attributes
/************************************************************************/
#define ACC1_ADDR				0x1D	// I2C address for first accelerometer
#define SCALE					0x08	// Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
#define DATARATE				0x07	// 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
#define SLEEPRATE				0x03	// 0=50Hz, 1=12.5, 2=6.25, 3=1.56
#define ASLP_TIMEOUT 			10		// Sleep timeout value until SLEEP ODR if no activity detected 640ms/LSB
#define MOTION_THRESHOLD		16		// 0.063g/LSB for MT interrupt (16 is minimum to overcome gravity effects)
#define MOTION_DEBOUNCE_COUNT 	1		// IN LP MODE, TIME STEP IS 160ms/COUNT
#define I2C_RATE				100

#define NUM_AXIS 3
#define NUM_ACC_DATA (DATARATE * NUM_AXIS)

#define ALL_INTS 3

typedef enum{
	F800 = 0,
	F400,
	F200,
	F100,
	F50,
	F12,
	F6,
	F1
} ACC_ODR;


/************************************************************************/
//PROTOTYPES
/************************************************************************/
void initialize_pins(void);
long read_batt(void);
void clear_acc_ints(void);
void disable_int(uint8_t pcie);
void enable_int(uint8_t pcie);
void set_factory_mode(void);
void sleep_handler(bool still);
void setup(void);
void loop(void);


/************************************************************************/
//GLOBAL VARIABLES
/************************************************************************/
volatile bool got_slp_wake;
volatile bool got_data_acc;
volatile bool factory_sleep;
volatile bool got_int_ble;
bool accel_on;
bool _sleep_inactive;
bool _sleep_active;

volatile uint8_t int_source = 0x00;

uint16_t intCount;
int accelCount[3];  				// Stores the 12-bit signed value
float accelG[3];  					// Stores the real accel value in g's
MMA8452 acc1 = MMA8452(ACC1_ADDR);

/************************************************************************/

void setup()
{
	//initialize the pins
	initialize_pins();
	
	sei();								//ENABLE EXTERNAL INTERRUPT FOR WAKEUP
	got_slp_wake = false;
	got_data_acc = false;
	got_int_ble = false;
	_sleep_inactive = false;
	_sleep_active = false;
	
	intCount = 0;
	
#ifdef RDV_DEBUG
	Serial.begin(115200);
	while(!Serial);
#endif
	
	acc1.setBitrate(I2C_RATE);
	
	//while(1){
		//i2cSendStart();
		//i2cSendByte(0xAA);
		//i2cSendStop();
		//delayMicroseconds(100);
	//}
	
#ifdef RDV_DEBUG
	Serial.print("I2C Speed: ");
	Serial.println(I2C_RATE);
#endif
	
	if (acc1.readRegister(WHO_AM_I) == 0x2A) 						// WHO_AM_I should always be 0x2A
	{
		accel_on = true;

#ifdef RDV_DEBUG
		Serial.println("MMA8452Q is on-line...");
#endif
	}
	else
	{
#ifdef RDV_DEBUG
		Serial.println("Could not connect to MMA8452Q");
#endif
		accel_on = false;
	}
	
#ifdef RDV_DEBUG
	Serial.println("WELCOME TO BLINDSAFE...");
	Serial.println("+++++++++++++++++++++++++++++");
	long tempVcc = read_batt();
	Serial.print("Power Level At: ");
	Serial.print(tempVcc / 1000);
	Serial.print(".");
	Serial.print(tempVcc % 1000);
	Serial.println("V");
	Serial.print("ACC ODR: ");
	Serial.println(DATARATE);
	Serial.print("ACC 1: ");
	Serial.println(accel_on);
	Serial.println("+++++++++++++++++++++++++++++");
	Serial.println("Setting into factory mode...");
	delay(2500);
#endif
	set_factory_mode();
}


// Main loop
void loop()
{
	//BLE SENT INTERRUPT
	if(got_int_ble)
	{
		got_int_ble = false;
		
#ifdef RDV_DEBUG
		Serial.println("Setting up accelerometers...");
#endif
		enable_int(ALL_INTS);

		acc1.initMMA8452(SCALE, DATARATE, SLEEPRATE, ASLP_COUNT, MOTION_THRESHOLD, MOTION_DEBOUNCE_COUNT);
	}
	
	//ACCELEROMETER DATA IS READY
	if (got_data_acc && accel_on)
	{
		acc1.readAccelData(accelCount);  // Read the x/y/z adc values, clears int
		got_data_acc = false;
		
		// Now we'll calculate the acceleration value into actual g's
		for (int i=0; i<3; i++)
		accelG[i] = (float) accelCount[i]/((1<<12)/(2*SCALE));  // get actual g value, this depends on scale being set
		// Print out values

		for (int i=0; i<NUM_AXIS; i++)
		{
#ifdef RDV_DEBUG
			Serial.print(accelG[i], 4);  // Print g values
			Serial.print("\t");  // tabs in between axes
#endif
		}
#ifdef RDV_DEBUG
		Serial.println();
#endif
	}
	
	//ACCELEROMETER WENT INTO SLEEP/AWAKE MODE
	if(got_slp_wake)
	{
		got_slp_wake = false;
		
		#ifdef RDV_DEBUG
		Serial.print("INT 1: 0x");
		Serial.println(int_source, HEX);
		#endif
		switch(int_source)
		{
			case 0x84:		//MOTION AND SLEEP/WAKE INTERRUPT (if even possible?)
			case 0x80:		//SLEEP/WAKE INTERRUPT
			{
				uint8_t sysmod = acc1.readRegister(SYSMOD);
				//acc1.readRegister(FF_MT_SRC);	//CLEAR MOTION INTERRUPT
#ifdef RDV_DEBUG
				Serial.print("SYSMOD: 0x");
				Serial.println(sysmod);
#endif
				if(sysmod == 0x02)    		//SLEEP MODE
				_sleep_inactive = true;
				else if(sysmod == 0x01)  	//WAKE MODE
				_sleep_inactive = false;
				break;
			}
			case 0x04:						//MOTION INTERRUPT
			default:
			break;
		}
#ifdef RDV_DEBUG
		Serial.println("Clearing ints...");
#endif
		clear_acc_ints();			//clear interrupts at the end of this handler
	}

	if(_sleep_inactive)
	{
#ifdef RDV_DEBUG
		Serial.flush();	 //clear the buffer before sleeping, otherwise can lock up the pipe
		delay(10);
#endif
		sleep_handler(true);
	}
	else if(_sleep_active)
	{
#ifdef RDV_DEBUG
		Serial.flush();	 //clear the buffer before sleeping, otherwise can lock up the pipe
		delay(10);
#endif
		sleep_handler(false);
	}
}// void loop()


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	**NOT USED IN PRODUCTION**
//#					Puts the blind spot detector into factory mode.  This mode
//#		 			is essentially an ultra deep sleep mode that is only brought out
//#					of sleep by one specific interrupt generated.
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void set_factory_mode()
{
	got_slp_wake = false;
	got_data_acc = false;
	got_int_ble = false;
	factory_sleep = true;
	
	enable_int(PCIE0);
	disable_int(PCIE1);
	disable_int(PCIE2);
	acc1.MMA8452Standby();
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	cli();
	sleep_bod_disable();
	sei();
	sleep_cpu();
	sleep_disable();
	clear_acc_ints();
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Creates a new deck with a specified number of cards, logging
//#		 			the cards initial position for comparison after rounds later.
//#
//# Parameters:		Deck structure pointer with the specified deck length and pointers
//#					to the head and tail of the two "piles"
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void sleep_handler(bool still)
{
	got_slp_wake = false;
	got_data_acc = false;
	got_int_ble = false;
	factory_sleep = false;

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	cli();
	sleep_bod_disable();
	enable_int(PCIE0);
	enable_int(PCIE1);
	disable_int(PCIE2);
	still ? disable_int(PCIE2) : enable_int(PCIE2);	//we want to sleep in between data reads AND when no motion occurs
	clear_acc_ints();
	sei();
	sleep_cpu();
	sleep_disable();
	enable_int(PCIE2);
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Reads the specified registers to clear all interrupts for the
//#					accelerometer.
//#					INT_SOURCE | FF_MT_SRC | ACCELEROMETER DATA
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void clear_acc_ints()
{
	acc1.readRegister(INT_SOURCE);
	acc1.readRegister(FF_MT_SRC);
	acc1.readAccelData(accelCount);
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Sets up the pin configurations
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void initialize_pins()
{
	//outputs
	DDRB =  (1 << LED_CNTL) | (1 << CE_REG1) | (1 << CE_REG);
	DDRC =  (1 << BR_CNTL);
	DDRD =  (1 << TRIGGER);
	//inputs
	DDRB =  ~(1 << INT1_ACC);
	DDRC =  ~(1 << VCC_SOLAR) & ~(1 << BATT_READ) & ~(1 << INT2_ACC);
	DDRD =  ~(1 << ECHO_3V3) & ~(1 << SOFT_SDA) & ~(1 << SOFT_SCL);
	//pwm setup
	//TODO
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Enables the corresponding interrupt bank.
//#
//# Parameters:		Interrupt bank to enable.  Default is all interrupts enabled.
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void enable_int(uint8_t pcie)
{
	switch(pcie)
	{
		case PCIE0:
		{
			PCMSK0 |= (1 << PCINT4);	//INT1_ACC Interrupt
			break;
		}
		case PCIE1:
		{
			PCMSK1 |= (1 << PCINT11);	//INT2_ACC Interrupt
			break;
		}
		case PCIE2:
		{
			PCMSK2 |= (1 << PCINT18);	//Undefined
			break;
		}
		default:
		{
			PCMSK0 |= (1 << PCINT4);	//BLE Interrupt
			PCMSK1 |= (1 << PCINT11);	//ACC SLEEP/WAKE interrupt
			PCMSK2 |= (1 << PCINT18);	//ACC DATA interrupt
			break;
		}
	}
	
	if(pcie <= PCIE2)
		PCICR |= (1 << pcie);
	else
		PCICR |= ((1 << PCIE0) | (1 << PCIE1) | (1 << PCIE2));
}


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Disables the corresponding interrupt bank.
//#
//# Parameters:		Interrupt bank to disable.  Default is all interrupts disabled.
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void disable_int(uint8_t pcie)
{
	switch(pcie)
	{
		case PCIE0:
		{
			PCMSK0 &= ~(1 << PCINT4);	//INT1_ACC Interrupt
			break;
		}
		case PCIE1:
		{
			PCMSK1 &= ~(1 << PCINT11);	//INT2_ACC Interrupt
			break;
		}
		case PCIE2:
		{
			PCMSK2 &= ~(1 << PCINT18);	//Undefined
			break;
		}
		default:
		{
			PCMSK0 &= ~(1 << PCINT4);	
			PCMSK1 &= ~(1 << PCINT11);	
			PCMSK2 &= ~(1 << PCINT18);	
			break;
		}
	}
	
	if(pcie <= PCIE2)
		PCICR &= ~(1 << pcie);
	else
		PCICR = 0;
	
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Reads the internal bandgap to get an accurate battery power
//#					level reading.
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
long read_batt()
{
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	cli();
	uint8_t ADC_state = ADMUX;
	
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (ADCSRA & _BV(ADSC)); // measuring
	
	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
	uint8_t high = ADCH; // unlocks both
	
	long result = (high<<8) | low;
	
	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	
	ADMUX = ADC_state;
	sei();
	
	return result; // Vcc in millivolts
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Creates a new deck with a specified number of cards, logging
//#		 			the cards initial position for comparison after rounds later.
//#
//# Parameters:		Deck structure pointer with the specified deck length and pointers
//#					to the head and tail of the two "piles"
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER//////////////////////////////////////////////////////
ISR(PCINT1_vect)
{
	cli();
	if(digitalRead(INT1_ACC))
	{
		int_source= acc1.readRegister(INT_SOURCE) & 0xFE; //we don't care about the data interrupt here
		acc1.readRegister(FF_MT_SRC);	//CLEAR MOTION INTERRUPT
		got_slp_wake = true;
	}
	sei();
}


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Creates a new deck with a specified number of cards, logging
//#		 			the cards initial position for comparison after rounds later.
//#
//# Parameters:		Deck structure pointer with the specified deck length and pointers
//#					to the head and tail of the two "piles"
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
ISR(PCINT2_vect)
{
	cli();
	if(digitalRead(INT2_ACC))
	{
		got_data_acc = true;
	}
	sei();
}
