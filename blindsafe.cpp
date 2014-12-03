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
#include "Ultrasonic.h"
#include "NewPing.h"
#include "MMA8452.h"
#include <avr/sleep.h>
#include <util/delay.h>

#define RDV_DEBUG

/************************************************************************/
//PIN/PORT DEFINITIONS: 
/************************************************************************/
#define CE_REG			PORTB1	//OUTPUT SOLAR PANEL OUTPUT
#define INT1_ACC		PORTB2	//INPUT INTERRUPT MOTION DETECTION (PCIE0)
#define VCC_SOLAR		PORTC0	//INPUT ADC 
#define BATT_READ		PORTC1	//INPUT ADC 
#define BATT_RD_CNTL	PORTC2	//OUTPUT
#define INT2_ACC		PORTC3	//INPUT INTERRUPT DATA READY (PCIE1)
#define LED_CNTL1		PORTD2	//OUTPUT PWM AMBER LED
//#define TWO_LEDS			//TODO: define this when new LED is in place
//#define BRIGHTNESS_CTNL	//TODO: define this if you want to control brightness based on daylight
#define LED_CNTL2		PORTD1	//OUTPUT PWM RED LED		//TODO: add this pin somewhere
#define CE_REG1			PORTD4	//OUTPUT SENSOR AND LED OUTPUT
#define ECHO_3V3		PORTD5	//INPUT
#define TRIGGER			PORTD6	//OUTPUT

#define CE_REG_PORT		PORTB
#define LED1_PORT		PORTD
#define LED2_PORT		PORTD
//#define LED1_PIN		ARDUINO PIN
//#define LED2_PIN		ARDUINO PIN

#define MAX_SONAR_DISTANCE 500
#define MIN_SONAR_DISTANCE 100
#define TRIGGER_PIN		6			
#define ECHO_PIN		5
//#define TRIGGER_PORT	PORTD
//#define ECHO_PORT		PORTD
//#define TRIGGER_DDR	DDRD
//#define ECHO_DDR		DDRD

#define INT1_PCINT		PCINT2
#define INT2_PCINT		PCINT11
#define INT1_PCIE		PCIE0
#define INT2_PCIE		PCIE1
#define INT1_PCMSK		PCMSK0
#define INT2_PCMSK		PCMSK1
#define INT1_PIN		PINB
#define INT2_PIN		PINC
#define ALL_INTS		2

//BIT-BANG I2C PINS (define in i2c.h)
#define SOFT_SCL		PORTD6	//INOUT
#define SOFT_SDA		PORTD7	//INOUT

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
//BATTERY/SOLAR CONSTANTS:
/************************************************************************/
#define LOW_BATTERY				3.5f
#define CHARGED_BATTERY			3.8f
#define DAYTIME					2.0f
#define BATTERY_CHECK_INTERVAL  300

/************************************************************************/
//DATA STRUCTURES:
/************************************************************************/
typedef struct battery_t{
	bool daylight;
	long solar_vcc;
	long battery_vcc;
	long mcu_vcc;
	uint8_t brightness;
}Battery;

/************************************************************************/
//PROTOTYPES
/************************************************************************/
//void handle_battery_mgmt(void);
void ISR_notify_timer();
void wake_motion_slp(void);
//void check_motion(void);
void init_accelerometer(void);
void initialize_pins(void);
long read_mcu_batt(void);
bool clear_acc_ints(void);
void disable_int(uint8_t pcie);
void enable_int(uint8_t pcie);
void deep_sleep_handler(bool still);
void setup(void);
void loop(void);


/************************************************************************/
//GLOBAL VARIABLES
/************************************************************************/
static volatile bool got_slp_wake;
static volatile bool got_data_acc;
static volatile bool _sleep;
static volatile bool time_up;
static volatile uint8_t intSource;
static volatile bool driving;
static volatile uint16_t batt_counter;
static bool accel_on;

static unsigned long usec;
static float range;
static int16_t accelCount[3];  				// Stores the 12-bit signed value
static float accelG[3];  						// Stores the real accel value in g's
AccOdr f_odr = (AccOdr)DATARATE;
MMA8452 accel = MMA8452(ACCEL_ADDR);
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
Battery battery;

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
	
	//setup the led timer
	NewPing::timer_ms(600, ISR_notify_timer);
	//initialize battery monitoring system
	battery.battery_vcc = 0;
	battery.solar_vcc = 0;
	battery.daylight = false;
	battery.mcu_vcc = 0;
	battery.brightness = 255;	//default to full brightness
	batt_counter = 0;
	ADCSRA = 0;	//disable ADC
	
	sei();								//ENABLE EXTERNAL INTERRUPT FOR WAKEUP
	got_slp_wake = false;
	got_data_acc = false;
	_sleep = false;
	driving = false;
	
	
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
	battery.mcu_vcc = read_mcu_batt();
	Serial.print("Power Level At: ");
	Serial.print(battery.mcu_vcc / 1000);
	Serial.print(".");
	Serial.print(battery.mcu_vcc % 1000);
	Serial.println("V");
	Serial.print("ACC ODR: ");
	Serial.println(f_odr);
	Serial.print("ACC 1: ");
	Serial.println(accel_on);
	Serial.println("+++++++++++++++++++++++++++++");
	//Serial.println("Setting into factory mode...");
	delay(500);
#endif
	init_accelerometer();
	
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
		
		//sequentially check for motion detection
		if(INT1_PIN & (1 << INT1_ACC) & check_still())
			driving = false;	//we will go through one more time before, maybe break out here?
		else 
			driving = true;
			
#ifdef RDV_DEBUG
		Serial.print("Driving: ");
		Serial.println(driving ? "Yes" : "No");
#endif
		_sleep = true;
		
		if(++batt_counter > BATTERY_CHECK_INTERVAL)
			handle_battery_mgmt();
		
		CE_REG_PORT |= (1 << CE_REG);	//enable power to sonar and LED
		delayMicroseconds(5);	//TODO: may need to adjust delay here to optimize bringup time
		disable_int(ALL_INTS);	
		range = 0;
		usec = ultrasonic.timing();
		range = ultrasonic.convert(usec, ultrasonic.CM);
#ifdef RDV_DEBUG
		Serial.print("Distance: ");
		Serial.print(range);
		Serial.println("cm");
#endif
		if(range < MAX_SONAR_DISTANCE && range >= MIN_SONAR_DISTANCE) 
		{
#ifdef TWO_LEDS
			if(Battery.battery_vcc < LOW_BATTERY)
				LED2_PORT |= (1 << LED_CNTL2);
			else
				LED1_PORT |= (1 << LED_CNTL1);
#else
			LED1_PORT |= (1 << LED_CNTL1);
#endif

//TODO: brightness control
//#ifdef BRIGHTNESS_CTNL
//#ifdef TWO_LEDS
			//if(Battery.battery_vcc < LOW_BATTERY)
				//analogWrite(LED2_PIN, Battery.brightness);
			//else
				//analogWrite(LED1_PIN, Battery.brightness);
//#else
			//analogWrite(LED1_PIN, Battery.brightness);
//#endif			
//#endif
		}
		
		//wait for timer to expire
		time_up = false;
		//TODO: can put POWER_SAVE_MODE type sleep here if IO can remain on
		while(!time_up);
		time_up = false;
		
		//turn off the led
#ifdef TWO_LEDS
		if(Battery.battery_vcc < LOW_BATTERY)
			LED2_PORT &= ~(1 << LED_CNTL2);
		else
			LED1_PORT &= ~(1 << LED_CNTL1);
#else
		LED1_PORT &= ~(1 << LED_CNTL1);
#endif
		
		deep_sleep_handler(driving);
		
#ifdef RDV_DEBUG		
		// Now we'll calculate the acceleration value into actual g's
		for (uint16_t i=0; i<3; i++)
		accelG[i] = (float) accelCount[i]/((1<<12)/(2*SCALE));  // get actual g value, this depends on scale being set
		// Print out values

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
		driving = false;		//sleep without waking up from data interrupt
	}
		
	if(_sleep)
	{
#ifdef RDV_DEBUG
		Serial.print("Sleep Active = ");
		Serial.println(driving);
		Serial.flush();	 //clear the buffer before sleeping, otherwise can lock up the pipe
#endif
		deep_sleep_handler(driving);
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
void init_accelerometer(){
	
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
void deep_sleep_handler(bool active)
{
	got_slp_wake = false;
	got_data_acc = false;
	ADCSRA = 0;	//disable ADC
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
	// turn off brown-out enable in software
	MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
	MCUCR = bit (BODS);        // this must be done within 4 clock cycles of above
	sei();
	sleep_cpu();
	sleep_disable();
	if(!active)
		enable_int(PCIE1);
	_sleep = false;
	//TODO: enable brown out
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
	//TODO: change the port values to match the new architecture
	//outputs
	DDRB =  (1 << CE_REG);
	DDRC =  (1 << BATT_RD_CNTL);
	DDRD =  (1 << TRIGGER) | (1 << CE_REG1) | (1 << LED_CNTL1);
	PORTB = 0;	//TODO: may need to make CE_REG 1 to keep charging on?? or can do it later before first sleep
	PORTC = 0;
	PORTD = 0;
	
	//inputs	
	DDRB &=  ~(1 << INT1_ACC);
	PORTB |= (1 << INT1_ACC);	//activate pullups
	
	DDRC &=  ~((1 << VCC_SOLAR) | (1 << BATT_READ) | (1 << INT2_ACC));
	PORTC |= (1 << INT2_ACC) | (1 << BATT_READ) | (1 << INT2_ACC);	
	
	DDRD &=  ~((1 << ECHO_3V3) | (1 << SOFT_SDA) | (1 << SOFT_SCL));
	PORTD |= (1 << ECHO_3V3);	//activate pullups except on I2C pins
	
	//TODO: PWM SETUP
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Is called every time the device comes out of sleep in 
//#					the parked/idle state
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void wake_motion_slp()
{	
	//detect daylight, read battery and execute power management
	handle_battery_mgmt();
	//turn off LED IO
#ifdef TWO_LEDS
	if(Battery.battery_vcc < LOW_BATTERY)
		LED2_PORT &= ~(1 << LED_CNTL2);
	else
		LED1_PORT &= ~(1 << LED_CNTL1);
#else
	LED1_PORT &= ~(1 << LED_CNTL1);
#endif
	
		
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Handles all battery management, including controlling the 
//#					solar voltage output and daylight/brightness detection, as well
//#					as the battery power level indication
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void handle_battery_mgmt()
{
	//need to turn on ADC
	//sense brightness on solar
	//read battery voltage
	//turn off solar if fully charged
	//turn off ADC
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	This checks to see if we are moving or not
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void check_still()
{
	bool still = false;
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
			motion = true;
			else if(sysmod == 0x01)  	//WAKE MODE
			motion = false;
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
	driving = false;		//sleep without waking up from data interrupt
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
long read_mcu_batt()
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

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	ISR for the newping timer that utilizes timer 2 to wake up
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void ISR_notify_timer() {
	cli();
	time_up = true;
	sei();
}