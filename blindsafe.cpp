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
#include <util/delay.h>

//#include "i2c.h"
#define RDV_DEBUG

//PIN DEFINITIONS

//blind spot detector pins
#define CE_REG		PORTB1	//OUTPUT
#define INT1_ACC	PORTB2	//INPUT INTERRUPT MOTION DETECTION (PCIE0)
#define VCC_SOLAR	PORTC0	//INPUT ADC 
#define BATT_READ	PORTC1	//INPUT ADC 
#define BATT_READ_CNTL		PORTC2	//OUTPUT
#define INT2_ACC	PORTC3	//INPUT INTERRUPT DATA READY (PCIE1)
#define LED_CNTL	PORTD2	//OUTPUT PWM
#define CE_REG1		PORTD4	//OUTPUT
#define ECHO_3V3	PORTD5	//INPUT
#define TRIGGER		PORTD6	//OUTPUT

#define INT1_PCINT	PCINT2
#define INT2_PCINT	PCINT11
#define INT1_PCIE	PCIE0
#define INT2_PCIE	PCIE1
#define INT1_PCMSK	PCMSK0
#define INT2_PCMSK	PCMSK1
#define INT1_PIN	PINB
#define INT2_PIN	PINC

//BIT-BANG I2C PINS (define in i2c.h)
#define SOFT_SCL	PORTD6	//INOUT
#define SOFT_SDA	PORTD7	//INOUT

/************************************************************************/
//ACCELEROMETER CONSTANTS: these all need to be translated to attributes
/************************************************************************/
#define ACCEL_ADDR				0x1D	// I2C address for first accelerometer
#define SCALE					0x08	// Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
#define DATARATE				0x07	// 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
#define SLEEPRATE				0x03	// 0=50Hz, 1=12.5, 2=6.25, 3=1.56
#define ASLP_TIMEOUT 			10		// Sleep timeout value until SLEEP ODR if no activity detected 640ms/LSB
#define MOTION_THRESHOLD		16		// 0.063g/LSB for MT interrupt (16 is minimum to overcome gravity effects)
#define MOTION_DEBOUNCE_COUNT 	1		// IN LP MODE, TIME STEP IS 160ms/COUNT
#define I2C_RATE				100

#define NUM_AXIS 3
#define NUM_ACC_DATA (DATARATE * NUM_AXIS)

#define ALL_INTS 2

typedef enum{
	F80000 = 0,
	F40000,
	F20000,
	F10000,
	F5000,
	F1250,
	F625,
	F156
} AccOdr;


/************************************************************************/
//PROTOTYPES
/************************************************************************/
void init_accel(void);
void initialize_pins(void);
long read_batt(void);
bool clear_acc_ints(void);
void disable_int(uint8_t pcie);
void enable_int(uint8_t pcie);
void sleep_handler(bool still);
void setup(void);
void loop(void);


/************************************************************************/
//GLOBAL VARIABLES
/************************************************************************/
static volatile bool got_slp_wake;
static volatile bool got_data_acc;
static volatile bool _sleep;
static volatile uint8_t intSource;
static volatile bool _sleep_active;

static bool accel_on;

static int16_t accelCount[3];  				// Stores the 12-bit signed value
static float accelG[3];  						// Stores the real accel value in g's
AccOdr f_odr = (AccOdr)DATARATE;
MMA8452 accel = MMA8452(ACCEL_ADDR);

/************************************************************************/


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Initialization function.  Configures IO pins, enables interrupts.
//#					Heartbeat checks the accelerometerS.
//#					in debug, it can print out initialization stats.
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void setup()
{
	//initialize the pins
	initialize_pins();
	
	sei();								//ENABLE EXTERNAL INTERRUPT FOR WAKEUP
	got_slp_wake = false;
	got_data_acc = false;
	_sleep = false;
	_sleep_active = false;
	
	
#ifdef RDV_DEBUG
	Serial.begin(115200);
	while(!Serial);
	Serial.flush();	 //clear the buffer before sleeping, otherwise can lock up the pipe
	delay(1000);
#endif

	
	if (accel.readRegister(WHO_AM_I) == 0x2A) 						// WHO_AM_I should always be 0x2A
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
	Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
	long tempVcc = read_batt();
	Serial.print("Power Level At: ");
	Serial.print(tempVcc / 1000);
	Serial.print(".");
	Serial.print(tempVcc % 1000);
	Serial.println("V");
	Serial.print("ACC ODR: ");
	Serial.println(f_odr);
	Serial.print("ACC 1: ");
	Serial.println(accel_on);
	Serial.println("+++++++++++++++++++++++++++++");
	//Serial.println("Setting into factory mode...");
	delay(500);
#endif
	init_accel();
	
}


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Main loop
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void loop()
{
	//ACCELEROMETER DATA IS READY
	if (got_data_acc && accel_on)
	{
		got_data_acc = false;
		accel.readAccelData(accelCount);  // Read the x/y/z adc values, clears int
		_sleep_active = true;
		_sleep = true;
		
		// Now we'll calculate the acceleration value into actual g's
		for (uint16_t i=0; i<3; i++)
		accelG[i] = (float) accelCount[i]/((1<<12)/(2*SCALE));  // get actual g value, this depends on scale being set
		// Print out values
		
#ifdef RDV_DEBUG
		for (uint16_t i=0; i<NUM_AXIS; i++)
		{
			Serial.print(accelG[i], 4);  // Print g values
			Serial.print("\t");  // tabs in between axes
		}
		Serial.println();
#endif
	}
		
	//ACCELEROMETER CHANGED INTO SLEEP/AWAKE STATE
	if(got_slp_wake)
	{
		got_slp_wake = false;
		intSource= accel.readRegister(INT_SOURCE) & 0xFE; //we don't care about the data interrupt here
		accel.readRegister(FF_MT_SRC);	//CLEAR MOTION INTERRUPT
#ifdef RDV_DEBUG
		Serial.print("INT 1: 0x");
		Serial.println(intSource, HEX);
#endif
		switch(intSource)
		{
			case 0x84:		//MOTION AND SLEEP/WAKE INTERRUPT (if even possible?)
			case 0x80:		//SLEEP/WAKE INTERRUPT
			{
				uint8_t sysmod = accel.readRegister(SYSMOD);
				//accel.readRegister(FF_MT_SRC);	//CLEAR MOTION INTERRUPT
#ifdef RDV_DEBUG
				Serial.print("SYSMOD: 0x");
				Serial.println(sysmod);
#endif
				if(sysmod == 0x02)    		//SLEEP MODE
				_sleep = true;
				else if(sysmod == 0x01)  	//WAKE MODE
				_sleep = false;
				break;
			}
			case 0x04:						//MOTION INTERRUPT
			default:
			break;
		}
#ifdef RDV_DEBUG
		Serial.println("Clearing interrupts...");
#endif
		clear_acc_ints();			//clear interrupts at the end of this handler
		_sleep_active = false;		//sleep without waking up from data interrupt
	}
		
	if(_sleep)
	{
#ifdef RDV_DEBUG
		Serial.print("Sleep Active = ");
		Serial.println(_sleep_active);
		Serial.flush();	 //clear the buffer before sleeping, otherwise can lock up the pipe
#endif
		sleep_handler(_sleep_active);
	}//if(got_slp_wake)

}// void loop()

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Initializes the accelerometer and enables the interrupts
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void init_accel(){
	
	enable_int(ALL_INTS);
#ifdef RDV_DEBUG
	Serial.println("Setting up accelerometers...");
	Serial.print("Scale: ");
	Serial.println(SCALE);
	Serial.print("Data odr: ");
	Serial.println(DATARATE);
	Serial.print("Sleep odr: ");
	Serial.println(SLEEPRATE);
	Serial.print("Asleep count: ");
	Serial.println(ASLP_COUNT);
	Serial.print("Motion threshold: ");
	Serial.println(MOTION_THRESHOLD);
	Serial.print("Debounce count: ");
	Serial.println(MOTION_DEBOUNCE_COUNT);
	Serial.println("+++++++++++++++++++++++++++++");
#endif
	
	accel.initMMA8452(SCALE, DATARATE, SLEEPRATE, ASLP_COUNT, MOTION_THRESHOLD, MOTION_DEBOUNCE_COUNT);
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Puts the unit into sleep while enabling proper interrupts to
//#					exit sleep mode.  In normal mode, we want to sleep in between
//#					data ready acquisitions to maximize power.  When no motion is present,
//#					we only want to be woken up by BLE or movement again, not data
//#					ready.
//#
//# Parameters:		still --> 	true = disable acc data interrupts
//#								false = enable acc data interrupts
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void sleep_handler(bool active)
{
	got_slp_wake = false;
	got_data_acc = false;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	cli();
	sleep_bod_disable();
	enable_int(INT1_PCIE);
	active ? enable_int(INT2_PCIE) : disable_int(INT2_PCIE);	//if we want to sleep in between data reads AND when no motion occurs
	if(!clear_acc_ints()){
		Serial.println("Clear failed");
	}
	else{
		Serial.println("Clear succeed");
	}
	sei();
	sleep_cpu();
	sleep_disable();
	if(!active)
		enable_int(PCIE1);
	_sleep = false;
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
bool clear_acc_ints()
{
	if(accel.readRegister(INT_SOURCE) == ~0u)
		return false;
	if(accel.readRegister(FF_MT_SRC) == ~0u)
		return false;
	accel.readAccelData(accelCount);
	return true;
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
	DDRB =  (1 << CE_REG);
	DDRC =  (1 << BATT_READ_CNTL);
	DDRD =  (1 << TRIGGER) | (1 << CE_REG1) | (1 << LED_CNTL);
	PORTB = 0;	//TODO: may need to make CE_REG 1 to keep charging on?? or can do it later before first sleep
	PORTC = 0;
	PORTD = 0;
	
	//inputs	
	DDRB &=  ~(1 << INT1_ACC);
	PORTB |= (1 << INT1_ACC);	//activate pullups
	
	DDRC &=  ~((1 << VCC_SOLAR) | (1 << BATT_READ) | (1 << INT2_ACC));
	PORTC |= (1 << INT2_ACC);	//only tri-state the interrupt, not the ADC pins
	
	DDRD &=  ~((1 << ECHO_3V3) | (1 << SOFT_SDA) | (1 << SOFT_SCL));
	PORTD |= (1 << ECHO_3V3);	//activate pullups except on I2C pins
	
	//TODO: PWM SETUP
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
		case INT1_PCIE:
		{
			INT1_PCMSK |= (1 << INT1_PCINT);	//INT1_ACC Interrupt
			break;
		}
		case INT2_PCIE:
		{
			INT2_PCMSK |= (1 << INT2_PCINT);	//INT2_ACC Interrupt
			break;
		}
		default:
		{
			INT1_PCMSK |= (1 << INT1_PCINT);
			INT2_PCMSK |= (1 << INT2_PCINT);	
			break;
		}
	}
	
	if(pcie <= INT2_PCIE)
		PCICR |= (1 << pcie);
	else
		PCICR |= ((1 << INT1_PCIE) | (1 << INT2_PCIE));
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
			INT1_PCMSK &= ~(1 << INT1_PCINT);	//INT1_ACC Interrupt
			break;
		}
		case PCIE1:
		{
			INT2_PCMSK &= ~(1 << INT2_PCINT);	//INT2_ACC Interrupt
			break;
		}
		default:
		{
			INT1_PCMSK &= ~(1 << INT1_PCINT);	
			INT2_PCMSK &= ~(1 << INT2_PCINT);	
			break;
		}
	}
	
	if(pcie <= INT2_PCINT)
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
	while (ADCSRA & _BV(ADSC)); 
	
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
//# Description: 	ISR for the PCINT0 bus interrupt.  This is an external interrupt from
//#					the accelerometer that is triggered by filtered motion or if the
//#					accelerometer is entering or exiting sleep mode.
//#
//# Parameters:		Interrupt vector
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
ISR(PCINT0_vect)
{
	cli();
	if((INT1_PIN & (1 << INT1_ACC)))
	{
		got_slp_wake = true;
	}
	sei();
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	ISR for the PCINT1 bus interrupt.  This is an external interrupt from
//#					the accelerometer that is triggered by the accelerometer data being ready.
//#
//# Parameters:		Interrupt vector
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
ISR(PCINT1_vect)
{
	cli();
	if((INT2_PIN & (1 << INT2_ACC)))
	{
		got_data_acc = true;
	}
	sei();
}