/*************************************************************************
Title:    MRBus eTape Interface
Authors:  Michael Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Michael Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbux_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbusInit mrbeeInit
#define mrbusPacketTransmit mrbeePacketTransmit
#endif

#include "mrbus.h"

extern uint8_t mrbus_activity;
extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;

#define TH_STATE_IDLE          0x00
#define TH_STATE_TRIGGER       0x10 
#define TH_STATE_WAIT          0x20
#define TH_STATE_READ          0x30
#define TH_STATE_SEND_PACKET   0x40
#define TH_STATE_XMIT_WAIT     0x50
#define TH_STATE_WAKE          0x60

#define TH_EE_NUM_AVG          0x10

uint8_t mrbus_dev_addr = 0;
uint8_t th_state = TH_STATE_IDLE;
uint8_t num_avg = 1;
uint16_t pkt_period = 20;

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 6;
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	} 
	else if ('W' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
		if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		else if ( (MRBUS_EE_DEVICE_UPDATE_L == mrbus_rx_buffer[6]) || (MRBUS_EE_DEVICE_UPDATE_H == mrbus_rx_buffer[6]) )
		{
		    pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
#if defined DHT11 || defined DHT22
			pkt_period = max(pkt_period, 20);
#endif
		}
		else if (TH_EE_NUM_AVG == mrbus_rx_buffer[6])
		{
			num_avg = eeprom_read_byte((uint8_t*)TH_EE_NUM_AVG);			
#if defined DHT11 || defined DHT22 || defined TMP275 || defined CPS150 || defined HYT221
			num_avg = 1;
#endif
		}
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	
	}
	else if ('R' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}
	else if ('V' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
    {
        // Version
        mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
        mrbus_tx_buffer[MRBUS_PKT_LEN] = 14;
        mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'v';
#ifdef MRBEE
        mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRELESS;
#else
        mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRED;
#endif
        mrbus_tx_buffer[7]  = SWREV; // Software Revision
        mrbus_tx_buffer[8]  = SWREV; // Software Revision
        mrbus_tx_buffer[9]  = SWREV; // Software Revision
        mrbus_tx_buffer[10]  = HWREV_MAJOR; // Hardware Major Revision
        mrbus_tx_buffer[11]  = HWREV_MINOR; // Hardware Minor Revision
        mrbus_tx_buffer[12] = 'T';
        mrbus_tx_buffer[13] = 'H';
        mrbus_state |= MRBUS_TX_PKT_READY;
        goto PktIgnore;
    }
	else if ('X' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}

	// FIXME:  Insert code here to handle incoming packets specific
	// to the device.

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
	return;	
}

volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

ISR(ADC_vect)
{
	busVoltage += ADC;
	if (++busVoltageCount >=8)
	{
		// Disable ADC
		ADCSRA &= ~(_BV(ADEN) | _BV(ADIE));	
	}
}

#ifdef CPS150
#include "avr-i2c-master.h"

#define CPS150_I2C_ADDR        (0x28<<1)

#define CPS150_STATUS_DATA_VALID    0x00
#define CPS150_STATUS_DATA_STALE    0x01

volatile uint8_t cps150_read_countdown=0;

void cps150_start_conversion()
{
    uint8_t msgBuf[2];
	memset(msgBuf, 0, sizeof(msgBuf));    
    msgBuf[0] = CPS150_I2C_ADDR;
    msgBuf[1] = 0x00;
    i2c_transmit(msgBuf, 1, 0);
    while(i2c_busy());
    cps150_read_countdown = 5;
}

/* From the datasheet:
An example of the 14-bit compensated pressure with a full scale range of 30 to 120kPa can be calculated as follows:
Pressure [kPa] = (Pressure High Byte [5:0] x 256 + Pressure Low Byte [7:0]) / 2^14 x 90 + 30

The 14-bit compensated temperature can be calculated as follows:
Temperature [°C] = (Temperature High Byte [7:0] x 64 + Temperature Low Byte [7:2] / 4) / 2^14 x 165 – 40

*/

uint8_t cps150_read_value(uint16_t* kelvinTemp, uint16_t* barometricPressure)
{
    uint8_t msgBuf[6];
	uint16_t tmpVal=0;
	memset(msgBuf, 0, sizeof(msgBuf));

    msgBuf[0] = CPS150_I2C_ADDR + 0x01;  // Read address
    i2c_transmit(msgBuf, 5, 0);
	i2c_receive(msgBuf, 5);

	/*
	The mathematics of pressure conversion
	The goal is an output in hectoPascals
	Pkpa = [Pval] * (90 / 16384) + 30
	// Multiply the constant by 4 so it's a divide by 2^16
	Pkpa = [Pval] * (360 / 65536) + 30
	// Multiply the top by 10 and the trailing constant offset by 10 to convert to hectoPascals from kiloPascals
	Phpa = [Pval] * (3600 / 65536) + 300
	*/
	
	tmpVal = ((((uint16_t)msgBuf[1]) & 0x3F)<<8) | ((uint16_t)msgBuf[2]);
	*barometricPressure = (((((uint32_t)tmpVal) * 3600)>>16) & 0xFFFF) + 300;

	//*barometricPressure = ((uint16_t)msgBuf[1])<<8 | (uint16_t)msgBuf[2];
	

	/*
	The mathematics of temperature conversion
	The goal is getting an output in 1/16ths of degrees Kelvin
	Tc = [Tval] * (165 / 16384) - 40
	// Multiply the constant by 4 so it's a divide by 2^16
	Tc = [Tval] * (660 / 65536) - 40
	// Multiply the top by 16 and the trailing constant offset by 16 to convert to 1/16th degrees C
	T16c = [Tval] * (10560 / 65536) - 640
	// Adjust the trailing constant to be in Kelvin rather than in C
	T16k = [Tval] * (10560 / 65536) - 640 + 4370
	T16k = [Tval] * (10560 / 65536) + 3730
	*/

	tmpVal = (((uint16_t)msgBuf[3])<<6) | ((((uint16_t)msgBuf[4])>>2) & 0x3F);
	*kelvinTemp = (((((uint32_t)tmpVal) * 10560)>>16) & 0xFFFF) + 3730;

	//*kelvinTemp = (((uint16_t)msgBuf[3])<<6) | (((uint16_t)msgBuf[4])>>2);
		
	return(msgBuf[0] & 0xC0);
}

#endif

#ifdef HYT221
#include "avr-i2c-master.h"

#define HYT221_I2C_ADDR        (0x28<<1)

#define HYT221_STATUS_DATA_VALID    0x00
#define HYT221_STATUS_DATA_STALE    0x01

volatile uint8_t hyt221_read_countdown=0;

void hyt221_start_conversion()
{
    uint8_t msgBuf[2];
	memset(msgBuf, 0, sizeof(msgBuf));    
    msgBuf[0] = HYT221_I2C_ADDR;
    msgBuf[1] = 0x00;
    i2c_transmit(msgBuf, 1, 0);
    while(i2c_busy());
    hyt221_read_countdown = 5;
}

/* From the datasheet:
An example of the 14-bit compensated pressure with a full scale range of 30 to 120kPa can be calculated as follows:
Pressure [kPa] = (Pressure High Byte [5:0] x 256 + Pressure Low Byte [7:0]) / 2^14 x 90 + 30

The 14-bit compensated temperature can be calculated as follows:

Temperature [°C] = (Temperature High Byte [7:0] x 64 + Temperature Low Byte [7:2] / 4) / 2^14 x 165 – 40

*/

uint8_t hyt221_read_value(uint16_t* kelvinTemp, uint16_t* relativeHumidity)
{
    uint8_t msgBuf[6];
	uint16_t tmpVal=0;
	memset(msgBuf, 0, sizeof(msgBuf));

    msgBuf[0] = HYT221_I2C_ADDR + 0x01;  // Read address
    i2c_transmit(msgBuf, 5, 0);
	i2c_receive(msgBuf, 5);

	/*
	The mathematics of pressure conversion
	The goal is an output in hectoPascals
	RH = [RHval] * (100 / 16384)
	// Output is in 1/2 percent RH
	RH*2 = [RHval] * (200 / 16384)	
	*/
	
	tmpVal = ((((uint16_t)msgBuf[1]) & 0x3F)<<8) | ((uint16_t)msgBuf[2]);
	*relativeHumidity = ((((uint32_t)tmpVal) * 200)>>14) & 0xFF;

	/*
	The mathematics of temperature conversion
	The goal is getting an output in 1/16ths of degrees Kelvin
	Tc = [Tval] * (165 / 16384) - 40
	// Multiply the constant by 4 so it's a divide by 2^16
	Tc = [Tval] * (660 / 65536) - 40
	// Multiply the top by 16 and the trailing constant offset by 16 to convert to 1/16th degrees C
	T16c = [Tval] * (10560 / 65536) - 640
	// Adjust the trailing constant to be in Kelvin rather than in C
	T16k = [Tval] * (10560 / 65536) - 640 + 4370
	T16k = [Tval] * (10560 / 65536) + 3730
	*/

	tmpVal = (((uint16_t)msgBuf[3])<<6) | ((((uint16_t)msgBuf[4])>>2) & 0x3F);
	*kelvinTemp = (((((uint32_t)tmpVal) * 10560)>>16) & 0xFFFF) + 3730;

	//*kelvinTemp = (((uint16_t)msgBuf[3])<<6) | (((uint16_t)msgBuf[4])>>2);
		
	return(msgBuf[0] & 0xC0);
}

#endif




#ifdef TMP275

#include "avr-i2c-master.h"

#define TMP275_PTR_TEMP_REG    0x00
#define TMP275_PTR_CONFIG_REG  0x01
#define TMP275_PTR_TEMP_L_REG  0x10
#define TMP275_PTR_TEMP_H_REG  0x11

volatile uint8_t tmp275_read_countdown=0;

void tmp275_start_conversion()
{
    uint8_t msgBuf[4];
    msgBuf[0] = 0x9E;
    msgBuf[1] = 0x01;
    msgBuf[2] = 0xE1;
    msgBuf[3] = 0xE1;
    i2c_transmit(msgBuf, 4, 0);
    while(i2c_busy());
    
    tmp275_read_countdown = 3;
}

uint16_t tmp275_read_value()
{
    uint8_t msgBuf[4];
    msgBuf[0] = 0x9E;
    msgBuf[1] = 0x00;
    i2c_transmit(msgBuf, 2, 1);
	while(i2c_busy());
    msgBuf[0] = 0x9F;
    i2c_transmit(msgBuf, 3, 0);
	i2c_receive(msgBuf, 3);
	
	return((((uint16_t)msgBuf[1])<<4) | (0x0F & (msgBuf[2]>>4)));
}

#endif

#if defined DHT11 || defined DHT22

volatile uint8_t dht11_bitnum=0;
volatile uint8_t dht11_data[5];
volatile uint8_t dht11_read_complete=3;

void initializeDHT11Timer()
{
	// Set up timer 1 for 100Hz interrupts
	TCNT2 = 0;
	OCR2A = 0xC2;
	TCCR2A = 0;
	TCCR2B = _BV(CS21);
}

ISR(PCINT1_vect)
{
	// This is used by the DHT11 code to read asynchronously to the main loop
	if(!(PINC & _BV(PC3)))
	{
		uint8_t timerval = TCNT2;
		// It was the falling edge.
		// 26-28ms in the timer indicates that it's a 0, 70ms indicates a 1
		// 0.7265625uS per count
		// A logic zero will be between 27 and 48
		// A logic one will be between 83 and 110
		
		TCNT2 = 0;
		if (timerval > 70 && timerval < 120)
		{
			// It's a one, put one in the correct place
			dht11_data[dht11_bitnum / 8] |= 1<<(7-(dht11_bitnum % 8));
		
		}
		dht11_bitnum++;
	}
	else
	{
		// It's the rising edge, indicating that we're at the beginning of a clock period
		// Reset the timer
		TCNT2 = 0;
		TIMSK2 |= _BV(TOIE2);
	}

	if(dht11_bitnum >= 40)
	{
		// Shut down the PCINT11 interrupt
		PCIFR |= _BV(PCIF1);
		PCICR &= ~_BV(PCIE1);
		TIMSK2 &= ~_BV(TOIE2);

		if ((dht11_data[0] + dht11_data[1] + dht11_data[2] + dht11_data[3]) == dht11_data[4])
			// We're done, indicate complete and shut down the interrupt
			dht11_read_complete = 1;
		else
			dht11_read_complete = 2; // Indicate failure - checksums didn't match

	}
}


ISR(TIMER2_OVF_vect)
{
	// We overflowed, meaning the DHT11 stopped responding correctly
	dht11_read_complete = 2;
	// Shut down Timer 2
	TIMSK2 &= ~_BV(TOIE2);
	// Shut down the PCINT11 interrupt
	PCIFR |= _BV(PCIF1);
	PCICR &= ~_BV(PCIE1);
}

void dht11_start_conversion()
{
	uint8_t countdown = 20;
	// Get triggery
	// Set DHT11 data wire to ground for 20ms

#ifdef DHT11
	countdown = 20;
#elif defined DHT22
	countdown = 4;
#endif

	initializeDHT11Timer();

	dht11_bitnum=0;
	memset((void*)dht11_data, 0, sizeof(dht11_data));
	dht11_read_complete=0;
	PORTC &= ~_BV(PC3);
	DDRC |= _BV(PC3);
	while (countdown-- != 0)
	{
		_delay_ms(1);
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();			
	}
	
	// Clear all the timer interrupts - chances are we've overflowed
	//  since the last time we started
	TCNT2 = 0;
	TIFR2 |= 0x07;
	// Clear the timer counter and enable the interrupt
	TIMSK2 |= _BV(TOIE2);

	DDRC &= ~_BV(PC3);
	TCNT2 = 0;

	// 20-40uS high after AVR releases the line
	// Wait for the low for up to 186uS
	while ((PINC & _BV(PC3)) && 0 == dht11_read_complete);
	
	// Failure, the DHT11 didn't respond, return
	if (0 != dht11_read_complete)
	{
		dht11_read_complete = 0x42;
		return;
	}

	TCNT2 = 0;
	// After going high for 20-40uS, the DHT11 should pull it low 
	// for 80uS, then high again for 80uS
	// Wait up to 186 uS for the high.
	while (!(PINC & _BV(PC3)) && 0 == dht11_read_complete);

	// Failure, the DHT11 didn't respond, return
	if (0 != dht11_bitnum)
	{
		dht11_read_complete = 0x62;
		return;
	}

	// 20-40uS high after AVR releases the line
	// Wait for the low for up to 186uS
	TCNT2 = 0;
	while ((PINC & _BV(PC3)) && 0 == dht11_read_complete);

	TCNT2 = 0;
	// Failure, the DHT11 didn't respond, return
	if (0 != dht11_read_complete)
	{
		dht11_read_complete = 0x82;
		return;
	}

	// If we're here, we're on the low ahead of the first bit.  
	// Enable pin change interrupt and go!
	PCMSK1 = _BV(PCINT11);
	PCIFR |= _BV(PCIF1);
	PCICR |= _BV(PCIE1);
}

#endif

#ifdef LOWPOWER

volatile uint8_t wdt_tripped=0;

ISR(WDT_vect) 
{
	wdt_tripped=1;  // set global volatile variable
}


uint16_t system_sleep(uint16_t sleep_decisecs)
{
	uint16_t slept = 0;

	// Sleep the XBee
	PORTD |= _BV(PD7);

	while(slept < sleep_decisecs)
	{
		uint16_t remaining_sleep = sleep_decisecs - slept;
		uint8_t planned_sleep = 80;
		uint8_t wdtcsr_bits = _BV(WDIF) | _BV(WDIE);

		if (remaining_sleep == 1)
		{
			wdtcsr_bits |= _BV(WDP1) | _BV(WDP0);
			planned_sleep = 1;
		}
		else if (remaining_sleep <= 3)
		{
			wdtcsr_bits |= _BV(WDP2);
			planned_sleep = 3;
		}
		else if (remaining_sleep <= 5)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP0);
			planned_sleep = 5;
		}
		else if (remaining_sleep <= 10)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1);
			planned_sleep = 10;
		}
		else if (remaining_sleep <= 20)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
			planned_sleep = 20;
		}
		else if (remaining_sleep <= 40)
		{
			wdtcsr_bits |= _BV(WDP3);
			planned_sleep = 40;
		}
		else
		{
			wdtcsr_bits |= _BV(WDP3) | _BV(WDP0);
			planned_sleep = 80;
		}

		// Procedure to reset watchdog and set it into interrupt mode only

		cli();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // set the type of sleep mode to use
		sleep_enable();                           // enable sleep mode
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = wdtcsr_bits;

        sei();

		wdt_tripped = 0;
		// Wrap this in a loop, so we go back to sleep unless the WDT woke us up
		while (0 == wdt_tripped)
        	sleep_cpu();

		wdt_reset();
		WDTCSR |= _BV(WDIE); // Restore WDT interrupt mode

		slept += planned_sleep;
	}

	sleep_disable();

#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	MCUSR &= ~(_BV(WDRF));
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	// Unsleep the XBee
	PORTD &= ~_BV(PD7);

	return(slept);
}

#endif

// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	//OCR0A = 0xC2;
	OCR0A = 0x6C; // Appropriate for 11.0592 MHz
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;

#ifdef CPS150
		if (cps150_read_countdown)
			cps150_read_countdown--;
#endif

#ifdef HYT221
		if (hyt221_read_countdown)
			hyt221_read_countdown--;
#endif

#ifdef TMP275
		if (tmp275_read_countdown)
			tmp275_read_countdown--;
#endif	

	}
}

// End of 100Hz timer



void init(void)
{
	// Kill watchdog
    MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	DDRD = _BV(PD7);
	PORTD = 0x7F;

	DDRB = 0;
	PORTB = 0xFF;

	DDRC = _BV(PC3);	
	PORTC = 0xFF;

	ACSR = _BV(ACD);
#if defined TMP275 || defined CPS150 || defined HYT221
	i2c_master_init();
#endif
	
	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);

	// Initialize MRBus packet update interval from EEPROM
	pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
#if defined DHT11 || defined DHT22
	pkt_period = max(pkt_period, 20);
#endif

	// Initialize averaging value from EEPROM
#if defined DHT11 || defined DHT22 || defined TMP275 || defined CPS150 || defined HYT221
	num_avg = 1;
#else
	num_avg = eeprom_read_byte((uint8_t*)TH_EE_NUM_AVG);
#endif
	
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC
}


int main(void)
{
	uint16_t kelvinTemp = 4370; // Expressed in 1/16ths K, base of 273.15 K
	uint16_t relHumidity = 0; // Expressed in 1/10ths % RH
	uint16_t barometricPressure = 0;
	uint8_t count=0;
#if defined DHT11 || defined DHT22
	uint8_t dht11_powerup_lockout = 1;
#else
	uint8_t dht11_powerup_lockout = 0;
#endif

	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();

	// Prep for initial 'v' packet - fake a 'V' request
	mrbus_rx_buffer[0] = 0xFF;
	mrbus_rx_buffer[1] = 0xFF;
	mrbus_rx_buffer[2] = 0x06;
	mrbus_rx_buffer[3] = 0x6E;
	mrbus_rx_buffer[4] = 0x7F;
	mrbus_rx_buffer[5] = 0x56;
    mrbus_state |= MRBUS_RX_PKT_READY;

	sei();	

	while (1)
	{
		wdt_reset();
#ifdef MRBEE
		mrbeePoll();
#endif


		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();

		// Notes of warning - DANGER, WILL ROBINSON!  DANGER!
		// DHT11/DHT22/RHT03 must *NOT* be accessed within the first second of being powered up
		// DHT11/DHT22/RHT03 must *NOT* be read more than once every 2 seconds
		// Both of these restrictions come from the datasheet

		if (dht11_powerup_lockout && decisecs > 12)
			dht11_powerup_lockout = 0;

		if ((TH_STATE_IDLE == th_state) && (decisecs >= pkt_period) && !dht11_powerup_lockout)
		{
			th_state = TH_STATE_TRIGGER;
			count = 0;
			decisecs = 0;
		}
		else if (TH_STATE_TRIGGER == th_state)
		{
#if defined DHT11 || defined DHT22
			dht11_start_conversion();
			PORTC |= _BV(PC5);
#elif defined TMP275
			tmp275_start_conversion();
#elif defined CPS150
			cps150_start_conversion();
#elif defined HYT221
			hyt221_start_conversion();
#elif defined DUMMY_SENSOR
			// Do nothing
#endif
			// Trigger an ADC conversion
			busVoltage = 0;
			busVoltageCount = 0;
			ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
			th_state = TH_STATE_WAIT;
		}
		else if (TH_STATE_WAIT == th_state)
		{
			uint8_t conversionComplete = 0;

#if defined DHT11 || defined DHT22
			if (dht11_read_complete)
				conversionComplete = 1;
#elif defined TMP275
			if (0 == tmp275_read_countdown)
				conversionComplete = 1;
#elif defined CPS150
			if (0 == cps150_read_countdown)
				conversionComplete = 1;
#elif defined HYT221
			if (0 == hyt221_read_countdown)
				conversionComplete = 1;
#elif defined DUMMY_SENSOR
			// Just wait a while
			_delay_us(200);
			conversionComplete = 1;
#endif
			if (conversionComplete && !(ADCSRA & _BV(ADEN)) )
			{
				PORTC &= ~_BV(PC5);
				th_state = TH_STATE_READ;
			}
		}
		else if (TH_STATE_READ == th_state)
		{
			uint16_t temp;
			
			kelvinTemp = 4370; // Expressed in 1/16ths K, base of 273.15 K
			relHumidity = 0; // Expressed in 1/10ths % RH		


			// This is where things get ugly
			// For the DHT11 data bytes:
			//   0: integer humidity %
			//   1: unused, reads as 0
			//   2: integer temperature in C, (high bit indicates negative temp)
			//   3: unused, reads as 0
			//   4: checksum (literally a sum of the first four bytes)
			// For the DHT22/RHT03 data bytes:
			//   0: high 8 bits of humidity, in 1/10ths of %
			//   1: low 8 bits of humidity, in 1/10ths of %
			//   2: high 8 bits of temp in C, in 1/10ths of degrees (high bit indicates negative temp)
			//   3: low 8 bits of temp in C, in 1/10ths of degrees
			//   4: checksum (literally a sum of the first four bytes)
			// There's no systemic way to tell these things apart, so we have to be configured one way or the other

#ifdef DHT11
			relHumidity = (uint16_t)dht11_data[0] * 2;
			temp = (uint16_t)(dht11_data[2] & 0x7F);
			temp <<= 4;
			if (dht11_data[2] & 0x80)
				kelvinTemp -= temp;
			else
				kelvinTemp += temp;

#elif defined DHT22
			relHumidity = (((uint16_t)dht11_data[0])<<8) + (uint16_t)dht11_data[1];
			temp = ((uint16_t)((dht11_data[2] & 0x7F))<<8)+(uint16_t)dht11_data[3];
			temp *= 8;
			temp /= 5;
			
			relHumidity /= 5;
			
			if (dht11_data[2] & 0x80)
				kelvinTemp -= temp;
			else
				kelvinTemp += temp;
#elif defined TMP275
			temp = tmp275_read_value();
			// Sign extend the 12 bit result
			if (temp & 0x0800)
				temp |= 0xF000;
			kelvinTemp += temp;
			
#elif defined CPS150
			cps150_read_value(&kelvinTemp, &barometricPressure);

#elif defined HYT221
			hyt221_read_value(&kelvinTemp, &relHumidity);

#elif defined DUMMY_SENSOR
			// Tada!  Both the temperature and humity are going to be 42
			kelvinTemp = 42;
			relHumidity = 42;
#endif
			// Div by 8, as we use 8 samples
			busVoltage = busVoltage >> 3;  

#ifdef LOWPOWER
			busVoltage = ((uint32_t)busVoltage * 33) >> 10;
#else			
			//At this point, we're at (Vbus/6) / 5 * 1024
			//So multiply by 300, divide by 1024, or multiply by 75 and divide by 256
			busVoltage = ((uint32_t)busVoltage * 75) >> 8;
#endif
			if(++count >= num_avg)
			{
				th_state = TH_STATE_SEND_PACKET;
				count = 0;
			} else {
				th_state = TH_STATE_TRIGGER;
			}
		}
		else if(th_state == TH_STATE_SEND_PACKET)
		{
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 11;
			mrbus_tx_buffer[5] = 'S';
			mrbus_tx_buffer[6] = 0;  // Status byte.  Lower three bits are sensor type, 000 = DHT11/DHT22/RHT03
			mrbus_tx_buffer[7] = (kelvinTemp >> 8);
			mrbus_tx_buffer[8] = kelvinTemp & 0xff;
			mrbus_tx_buffer[9] = relHumidity & 0xff;
			mrbus_tx_buffer[10] = (uint8_t)busVoltage;
#ifdef CPS150
			mrbus_tx_buffer[11] = (barometricPressure >> 8);
			mrbus_tx_buffer[12] = barometricPressure & 0xff;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 13;
#endif

			mrbus_state |= MRBUS_TX_PKT_READY;

#ifdef LOWPOWER
			th_state = TH_STATE_XMIT_WAIT;
#else
			th_state = TH_STATE_IDLE;
#endif
		}


#ifdef LOWPOWER
		if (TH_STATE_XMIT_WAIT == th_state 
			&& !(mrbus_state & (MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY)))
		{
			uint8_t i=0;
			// FIXME: this is crap
			// Need to actually see when we're done sending out of the XBEE
			// Also, do this as a loop, in case we're on a short watchdog
			for (i=0; i<25; i++)
			{
				wdt_reset();							
				_delay_ms(10);
			}

			decisecs += system_sleep(pkt_period);
			th_state = TH_STATE_WAKE;
		}
		
		if (TH_STATE_WAKE == th_state)
		{
			// FIXME:  Do wakeup stuff
			th_state = TH_STATE_IDLE;		
		}
		
#endif

		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

#ifndef MRBEE
			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			{
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
				{
					//clrwdt();
					_delay_ms(1);
					if (mrbus_state & MRBUS_RX_PKT_READY) 
						PktHandler();
				}
			}
#endif
		}
	}
}



