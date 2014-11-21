//#START_MODULE_HEADER////////////////////////////////////////////////////////
//#
//# Filename:		cyclops.cpp
//#
//# Description:	This program is the main program for cycleAT tire sensor
//#
//# Authors:     	Ross Yeager
//#
//#END_MODULE_HEADER//////////////////////////////////////////////////////////

	#include "Arduino.h"
	#include "MMA8452REGS.h"
	#include "MMA8452.h"
	#include <avr/sleep.h>

	#define RDV_DEBUG
	#define LONGPACKET
	
	//ATMEGA PINS
	#define INT1_ACC	A0
	#define INT2_ACC	6
	#define INT_BLE		8	
	#define RXI			0
	#define TXO			1
	#define AR_RX		2
	#define AR_TX		3
	
	#define SAME_BLE	A1
	#define LED			13
	//static const uint8_t MOSI = 11;
	//static const uint8_t MISO = 12;
	//static const uint8_t SCK  = 13;
	
	/************************************************************************/
	//ACCELEROMETER CONSTANTS: these all need to be translated to attributes
	/************************************************************************/
	#define ACC1_ADDR				0x1D	// I2C address for first accelerometer
	#define ACC2_ADDR				0x1C	// I2C address for second accelerometer
	#define SCALE					0x08	// Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
	#define DATARATE				0x07	// 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
	#define SLEEPRATE				0x03	// 0=50Hz, 1=12.5, 2=6.25, 3=1.56
	#define ASLP_TIMEOUT 			10		// Sleep timeout value until SLEEP ODR if no activity detected 640ms/LSB
	#define MOTION_THRESHOLD		16		// 0.063g/LSB for MT interrupt (16 is minimum to overcome gravity effects)
	#define MOTION_DEBOUNCE_COUNT 	1		// IN LP MODE, TIME STEP IS 160ms/COUNT

	#define ALL_INTS 3
	#define NUM_AXIS 3
	
#ifdef DATARATE
	#define NUM_ACC_DATA (DATARATE * NUM_AXIS)
#endif
	
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

#ifdef LONGPACKET
	typedef struct BleLongPacket_t{
		uint8_t startpkt;
		float pressure;
		float temperature;
		float acc1_data[NUM_ACC_DATA];
		float acc2_data[NUM_ACC_DATA];
		uint8_t endpkt;
	}BleLongPacket;
#else
	typedef struct BleRawPacket_t{
		uint8_t startpkt;
		int32_t pressure;
		int32_t temperature;
		int acc1_data[NUM_ACC_DATA];
		int acc2_data[NUM_ACC_DATA];
		uint8_t endpkt;
	}BleRawPacket;
#endif

	/************************************************************************/
	//PROTOTYPES
	/************************************************************************/
	bool send_packet(BleLongPacket * blePacket);
	void read_press_temp();
	long read_vcc();
	void clear_acc_ints();
	void disable_int(uint8_t pcie);
	void enable_int(uint8_t pcie);
	void set_factory_mode();
	void sleep_handler(bool still);
	void setup();
	void loop();
	
	
	/************************************************************************/
	//GLOBAL VARIABLES
	/************************************************************************/
	volatile bool got_slp_wake;
	volatile bool got_data_acc;
	volatile bool factory_sleep;
	volatile bool got_int_ble;
	bool accel_on;
	bool both_acc;
	bool _sleep_inactive;
	bool _sleep_active;
	
	volatile uint8_t intSource;
	
	AccOdr f_odr = (AccOdr)DATARATE;
	uint16_t intCount;
	int16_t accelCount[3];  				// Stores the 12-bit signed value
	float accelG[3];  						// Stores the real accel value in g's
	MMA8452 acc1 = MMA8452(ACC1_ADDR);
	MMA8452 acc2 = MMA8452(ACC2_ADDR);
	BleLongPacket BlePkt;
	/************************************************************************/


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#	
//# Description: 	Initialization function.  Configures IO pins, enables interrupts.
//#					Heartbeat checks the accelerometers and determines if there are 1
//#					or 2 active (in case we unstuff the second one in future) and sets.
//#					the device into factory mode.  In debug, it can print out initialization stats.
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
	void setup()
	{
		pinMode(LED, OUTPUT);
		pinMode(SAME_BLE, INPUT);
		pinMode(INT1_ACC, INPUT);
		pinMode(INT2_ACC, INPUT);
		pinMode(INT_BLE, INPUT);
		
#ifdef RDV_DEBUG
		Serial.begin(115200);
		while(!Serial);
#endif
		
		sei();								//ENABLE EXTERNAL INTERRUPT FOR WAKEUP
		got_slp_wake = false;
		got_data_acc = false;
		got_int_ble = false;
		both_acc = false;
		_sleep_inactive = false;
		_sleep_active = false;
		
		intCount = 0;
		
		// WHO_AM_I should always be 0x2A
		accel_on = acc1.readRegister(WHO_AM_I) == 0x2A;
		both_acc = acc2.readRegister(WHO_AM_I) == 0x2A;
		
#ifdef RDV_DEBUG
		if(accel_on)
		{
			Serial.println("MMA8452Q is on-line...");
		}
		else
		{
			Serial.print("Could not connect to MMA8452Q");
		}
#endif
		
#ifdef RDV_DEBUG	
		Serial.println("WELCOME TO CYCLEAT...");
		Serial.println("+++++++++++++++++++++++++++++");
		long tempVcc = read_vcc();
		Serial.print("Power Level At: ");
		Serial.print(tempVcc / 1000);
		Serial.print(".");
		Serial.print(tempVcc % 1000);
		Serial.println("V");
		Serial.print("ACC ODR: ");
		Serial.println(f_odr);
		Serial.print("ACC 1: ");
		Serial.print(accel_on);
		Serial.print("\t");
		Serial.print("ACC 2: ");
		Serial.println(both_acc);
		Serial.println("+++++++++++++++++++++++++++++");
		Serial.println("Setting into factory mode...");
		delay(2500);
#endif
		set_factory_mode();
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
		//BLE SENT INTERRUPT
		if(got_int_ble)  
		{
			got_int_ble = false;
			
			
#ifdef RDV_DEBUG
			Serial.println("Setting up accelerometers...");
#endif
			enable_int(ALL_INTS);

			acc1.initMMA8452(SCALE, DATARATE, SLEEPRATE, ASLP_COUNT, MOTION_THRESHOLD, MOTION_DEBOUNCE_COUNT);
			if(both_acc) 
			{
				acc2.initMMA8452(SCALE, DATARATE, SLEEPRATE, ASLP_COUNT, MOTION_THRESHOLD, MOTION_DEBOUNCE_COUNT);	
			}
			else
			{
				acc2.MMA8452Standby(); //if we don't need acc2, put it in STANDBY to save power (or attempt to even if not working)
			}
		}
		
		//ACCELEROMETER DATA IS READY
		if (got_data_acc && accel_on) 
		{
			acc1.readAccelData(accelCount);  // Read the x/y/z adc values, clears int
			got_data_acc = false;
			
			if(++intCount >= f_odr)
			{
				read_press_temp();
				intCount = 0;
				send_packet(&BlePkt);
			}
			
			// Now we'll calculate the acceleration value into actual g's
			for (uint16_t i=0; i<3; i++)
			accelG[i] = (float) accelCount[i]/((1<<12)/(2*SCALE));  // get actual g value, this depends on scale being set
			// Print out values

			for (uint16_t i=0; i<NUM_AXIS; i++)
			{
				BlePkt.acc1_data[intCount*NUM_AXIS + i] = accelG[i];
				BlePkt.acc2_data[intCount*NUM_AXIS + i] = 0;
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
			intSource= acc1.readRegister(INT_SOURCE) & 0xFE; //we don't care about the data interrupt here
			acc1.readRegister(FF_MT_SRC);	//CLEAR MOTION INTERRUPT
#ifdef RDV_DEBUG
			Serial.print("INT 1: 0x");
			Serial.println(intSource, HEX);
#endif
			switch(intSource)
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
	
#ifdef RDV_DEBUG
		Serial.flush();	 //clear the buffer before sleeping, otherwise can lock up the pipe
		delay(10);
#endif
		sleep_handler(_sleep_inactive);
	}// void loop()
	
	
//********************************************************************
//	Functions and ISRs
//********************************************************************

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#	
//# Description: 	
//#					Puts the sensor into factory mode.  This mode
//#		 			is essentially an ultra deep sleep mode that is only brought out
//#					of sleep by one specific interrupt generated by the BLE unit.  
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
		acc2.MMA8452Standby();
			
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
//# Description: 	Puts the unit into sleep while enabling proper interrupts to 
//#					exit sleep mode.  In normal mode, we want to sleep in between
//#					data ready aquisitions to maximize power.  When no motion is present,
//#					we only want to be woken up by BLE or movement again, not data
//#					ready.
//#
//# Parameters:		still --> 	true = disable acc data interrupts
//#								false = enable acc data interrupts
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
		still ? disable_int(PCIE2) : enable_int(PCIE2);	//if we want to sleep in between data reads AND when no motion occurs
		clear_acc_ints();
		sei();
		sleep_cpu();
		sleep_disable();
		enable_int(PCIE2);
	}
	
//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#	
//# Description: 	Reads the specified registers to clear all interrupts for the
//#					accelerometer.  This must be done after processing or the device
//#					will get stuck in sleep mode because the interrupt cannot re-assert
//#
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
	
	bool send_packet(BleLongPacket * blePacket)
	{
		//TODO: add to characteristic, send to BLE
		return true;
	}
	

	void read_press_temp()
	{
		cli();
#ifdef RDV_DEBUG
		Serial.println("READ PRESS/TEMP");
#endif
		BlePkt.pressure = 0;
		BlePkt.temperature = 0;
		sei();
		clear_acc_ints();
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
				PCMSK0 |= (1 << PCINT0);	//BLE Interrupt
				PCICR |= (1 << PCIE0);
				break;
			}
			case PCIE1:
			{
				PCMSK1 |= (1 << PCINT8);	//ACC SLEEP/WAKE interrupt
				PCICR |= (1 << PCIE1);		
				break;
			}
			case PCIE2:
			{
				PCMSK2 |= (1 << PCINT22);	//ACC DATA interrupt
				PCICR |= (1 << PCIE2);
				break;
			}
			default:
			{
				PCMSK0 |= (1 << PCINT0);	//BLE Interrupt
				PCMSK1 |= (1 << PCINT8);	//ACC SLEEP/WAKE interrupt
				PCMSK2 |= (1 << PCINT22);	//ACC DATA interrupt
				
				PCICR |= ((1 << PCIE0) | (1 << PCIE1) | (1 << PCIE2));

				break;
			}
		}
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
				PCMSK0 &= ~(1 << PCINT0);	//BLE Interrupt
				PCICR &= ~(1 << PCIE0);
				break;
			}
			case PCIE1:
			{
				PCMSK1 &= ~(1 << PCINT8);	//ACC SLEEP/WAKE interrupt
				PCICR &= ~(1 << PCIE1);
				break;
			}
			case PCIE2:
			{
				PCMSK2 &= ~(1 << PCINT22);	//ACC DATA interrupt
				PCICR &= ~(1 << PCIE2);
				break;
			}
			default:
			{
				PCMSK0 &= ~(1 << PCINT0);	//BLE Interrupt
				PCMSK1 &= ~(1 << PCINT8);	//ACC SLEEP/WAKE interrupt
				PCMSK2 &= ~(1 << PCINT22);	//ACC DATA interrupt
				
				PCICR = 0;
				break;
			}
		}
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
	long read_vcc() 
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
//#					the BLE MCU.  It only is utilized during factory mode.
//#
//# Parameters:		Interrupt vector
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////	
	ISR(PCINT0_vect)
	{
		cli();
		if(digitalRead(INT_BLE))
		{
			if(factory_sleep)
			{
				factory_sleep = false;
				got_int_ble = true;
			}
			else 
			{
#ifdef RDV_DEBUG
				Serial.println("BLE pushed, no factory");
#endif
			}
		}
		sei();
	}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#	
//# Description: 	ISR for the PCINT1 bus interrupt.  This is an external interrupt from 
//#					the accelerometer that is triggered by filtered motion or if the 
//#					accelerometer is entering or exiting sleep mode.
//#
//# Parameters:		Interrupt vector
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////		
	ISR(PCINT1_vect)
	{
		cli();
		if(digitalRead(INT1_ACC))
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
	ISR(PCINT2_vect)
	{
		cli();
		if(digitalRead(INT2_ACC))
		{
			got_data_acc = true;
		}
		sei();
	}
	