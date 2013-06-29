/*************************************************************************
Title:    MRBus eTape Interface
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes & Michael Petersen

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
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "avr-i2c-master.h"

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define MRBUS_TX_PKT_READY MRBEE_TX_PKT_READY
#define MRBUS_RX_PKT_READY MRBEE_RX_PKT_READY
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

uint8_t mrbus_dev_addr = 0;

#define STATE_IDLE          0x00
#define STATE_TRIGGER       0x10 
#define STATE_WAIT          0x20
#define STATE_READ          0x30
#define STATE_SEND_PACKET   0x40

uint8_t sm_state = STATE_IDLE;

#define EE_NUM_AVG          0x10

#define EE_ETAPE_FULL_RATIO_H   0x20  // value = 1000 * Rfull / Rref
#define EE_ETAPE_FULL_RATIO_L   0x21
#define EE_ETAPE_EMPTY_RATIO_H  0x22  // value = 1000 * Rempty / Rref
#define EE_ETAPE_EMPTY_RATIO_L  0x23
#define EE_TEMPCAL_OFFSET       0x24
#define EE_TEMPCAL_GAIN_H       0x25
#define EE_TEMPCAL_GAIN_L       0x26

uint8_t nodeFlags = 0;
uint8_t num_avg = 1;
uint16_t pkt_period = 10;
uint8_t tempcal_offset = 0;
uint16_t tempcal_gain = 0;
uint16_t etape_full_ratio = 0;
uint16_t etape_empty_ratio = 0;

void init(void);
uint8_t check2990Busy(uint8_t addr);
uint8_t readETape(uint8_t addr, uint16_t *vtop, uint16_t *vmid, uint16_t *temp, uint16_t *tempint, uint16_t *vaux);
uint16_t offsetGainAdjust(uint16_t t);
uint8_t decInHex(uint16_t temp);

#ifdef ACCURATE_TIMER

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

uint8_t ticks;
uint16_t decisecs;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	decisecs = 0;
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 += 0xF3CB;
	ticks++;
	if (ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}
}

#else

// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

uint8_t ticks;
uint8_t decisecs;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
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
	}
	PORTB ^= _BV(PB0);
}

// End of 100Hz timer

#endif

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
		init();
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
        mrbus_tx_buffer[MRBUS_PKT_LEN] = 13;
        mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'v';
#ifdef MRBEE
        mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRELESS;
#else
        mrbus_tx_buffer[6]  = MRBUS_VERSION_WIRED;
#endif
        mrbus_tx_buffer[7]  = SWREV; // Software Revision
        mrbus_tx_buffer[8]  = HWREV_MAJOR; // Hardware Major Revision
        mrbus_tx_buffer[9]  = HWREV_MINOR; // Hardware Minor Revision
        mrbus_tx_buffer[10] = 'R';
        mrbus_tx_buffer[11] = 'T';
        mrbus_tx_buffer[12] = 'S';
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

// config is register 0x01, the LTC2990 configuration byte

#define LTC2990_CONFIG_TEMP_CELSIUS         0x00
#define LTC2990_CONFIG_TEMP_KELVIN          0x80
#define LTC2990_CONFIG_CONTINUOUS_ACQ       0x00
#define LTC2990_CONFIG_SINGLE_ACQ           0x40

#define LTC2990_CONFIG_MEASURE_INT_T_ONLY   0x00
#define LTC2990_CONFIG_MEASURE_T1_V1_ONLY   0x08
#define LTC2990_CONFIG_MEASURE_T2_V3_ONLY   0x10
#define LTC2990_CONFIG_MEASURE_ALL          0x18

#define LTC2990_CONFIG_MODE_V1_V2_T2        0x00
#define LTC2990_CONFIG_MODE_V1_M_V2_T2      0x01
#define LTC2990_CONFIG_MODE_V1_M_V2_V3_V4   0x02
#define LTC2990_CONFIG_MODE_T1_V3_V4        0x03
#define LTC2990_CONFIG_MODE_T1_V3_M_V4      0x04
#define LTC2990_CONFIG_MODE_T1_T2           0x05
#define LTC2990_CONFIG_MODE_V1_M_V2_V3_M_V4 0x06
#define LTC2990_CONFIG_MODE_V1_V2_V3_V4     0x07


void ltc2990_init(uint8_t config)
{
    uint8_t msgBuf[4];
    
    msgBuf[0] = 0xEE;
    msgBuf[1] = 0x01;
    msgBuf[2] = config;
    msgBuf[3] = 0;
    i2c_transmit(msgBuf, 3, 0);
    while(i2c_busy());
}

void init(void)
{
	// Clear watchdog (in the case of an 'X' packet reset)
    MCUSR = 0;
	wdt_reset();
	wdt_disable();

    sm_state = STATE_IDLE;
    
	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);

	// Initialize MRBus packet update interval from EEPROM
	pkt_period = (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) | eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L);

	// Initialize averaging value from EEPROM
	num_avg = eeprom_read_byte((uint8_t*)EE_NUM_AVG);

	// Initialize calibration coefficients from EEPROM
	tempcal_offset = eeprom_read_byte((uint8_t*)EE_TEMPCAL_OFFSET);
	tempcal_gain = (eeprom_read_byte((uint8_t*)EE_TEMPCAL_GAIN_H) << 8) | eeprom_read_byte((uint8_t*)EE_TEMPCAL_GAIN_L);
	etape_full_ratio = (eeprom_read_byte((uint8_t*)EE_ETAPE_FULL_RATIO_H) << 8) | eeprom_read_byte((uint8_t*)EE_ETAPE_FULL_RATIO_L);
	etape_empty_ratio = (eeprom_read_byte((uint8_t*)EE_ETAPE_EMPTY_RATIO_H) << 8) | eeprom_read_byte((uint8_t*)EE_ETAPE_EMPTY_RATIO_L);

	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC0 input
	ADCSRA = 0x07;  // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC
}


int main(void)
{
    int count = 0;
    
    uint16_t vtopRead = 0, vmidRead = 0, vauxRead = 0;
    uint16_t tempextRead = 0, tempintRead = 0, vddRead = 0;
    uint32_t vtop = 0, vmid = 0, vaux = 0, tempext = 0, tempint = 0, vdd = 0;
    uint8_t status = 0;

	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();

	i2c_master_init();

	// Prep for initial 'v' packet - fake a 'V' request
	mrbus_rx_buffer[0] = 0xFF;
	mrbus_rx_buffer[1] = 0xFF;
	mrbus_rx_buffer[2] = 0x06;
	mrbus_rx_buffer[3] = 0x6E;
	mrbus_rx_buffer[4] = 0x7F;
	mrbus_rx_buffer[5] = 0x56;
    mrbus_state |= MRBUS_RX_PKT_READY;

	sei();	

    ltc2990_init(LTC2990_CONFIG_TEMP_KELVIN
		| LTC2990_CONFIG_SINGLE_ACQ
        | LTC2990_CONFIG_MEASURE_ALL    
        | LTC2990_CONFIG_MODE_V1_V2_T2);   

	while (1)
	{
#ifdef MRBEE
		mrbeePoll();
#endif
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();
			
        if (decisecs >= pkt_period)
        {
            if(sm_state == STATE_IDLE)
            {
                sm_state = STATE_TRIGGER;
            }
            decisecs = 0;
        }

        if(sm_state == STATE_TRIGGER)
        {
            uint8_t msgBuf[2];

            // Trigger a 2990 conversion
            msgBuf[0] = 0xEE;
            msgBuf[1] = 0x02;
            i2c_transmit(msgBuf, 2, 0);
            while(i2c_busy());
            
            // Trigger an ADC conversion
            ADCSRA |= _BV(ADEN) | _BV(ADSC);
            
            sm_state = STATE_WAIT;
        }
        else if(sm_state == STATE_WAIT)
        {
            if( !check2990Busy(0x98) && !(ADCSRA & _BV(ADSC)) )
            {
                sm_state = STATE_READ;
            }
        }
        else if(sm_state == STATE_READ)
        {
            status = readETape(0x98, &vtopRead, &vmidRead, &tempextRead, &tempintRead, &vauxRead);
            vtop += vtopRead;
            vmid += vmidRead;
            tempext += tempextRead;
            tempint += tempintRead;
            vaux += vauxRead;
            
            vddRead = ADCL + (ADCH << 8);
            vdd += vddRead;
            
            count++;
            if(count >= num_avg)
            {
                sm_state = STATE_SEND_PACKET;
            	count = 0;
			}
			else
			{
			    sm_state = STATE_TRIGGER;
			}
        }
        else if(sm_state == STATE_SEND_PACKET)
		{
		    vtop /= num_avg;
		    vmid /= num_avg;
		    tempext /= num_avg;
		    tempint /= num_avg;
		    vaux /= num_avg;
		    vdd /= num_avg;
		    
		    tempext = offsetGainAdjust((uint16_t)tempext);
		    
		    uint16_t level = 0;
		    uint16_t ratio = (1000 * vmid) / (vtop - vmid);  // ratio in deci-percent
            if(ratio > etape_full_ratio)
            {
    		    level = 100 - (100 * (uint32_t)(ratio - etape_full_ratio)) / (etape_empty_ratio - etape_full_ratio);
            }
            else
            {
                level = 100;
            }
		    
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 15;
			mrbus_tx_buffer[5]  = 'S';
			mrbus_tx_buffer[6]  = status;
			mrbus_tx_buffer[7]  = level;
			mrbus_tx_buffer[8]  = ((tempext) >> 8) & 0xFF;
			mrbus_tx_buffer[9]  = (tempext) & 0xFF;
			mrbus_tx_buffer[10] = ((tempint) >> 8) & 0xFF;
			mrbus_tx_buffer[11] = (tempint) & 0xFF;
            mrbus_tx_buffer[12] = (VINDIV * VDD * vdd) / 1024;  // VINDIV is reciprocal of VIN divider ratio.  VDD is in decivolts
			mrbus_tx_buffer[13] = 0;
			mrbus_tx_buffer[14] = decInHex(level);

			vtop = 0;
			vmid = 0;
			tempext = 0;
			tempint = 0;
			vaux = 0;
			vdd = 0;
			
			mrbus_state |= MRBUS_TX_PKT_READY;
            sm_state = STATE_IDLE;
            ADCSRA &= ~(_BV(ADEN) | _BV(ADSC));  // Disable ADC
		}

		
		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			uint8_t bus_countdown;

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
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
				if (mrbus_state & MRBUS_RX_PKT_READY) 
					PktHandler();
			}
#endif
		}
	}
}


uint8_t check2990Busy(uint8_t addr)
{
    uint8_t msgBuf[10];
	msgBuf[0] = addr;
	msgBuf[1] = 0x00;
	i2c_transmit(msgBuf, 2, 1);
    while(i2c_busy());

	msgBuf[0] = addr | 0x01;
	i2c_transmit(msgBuf, 2, 0);
    while(i2c_busy());
	i2c_receive(msgBuf, 2);

    return (msgBuf[1] & 0x01);  // True if busy
    
}


uint8_t readETape(uint8_t addr, uint16_t *vtop, uint16_t *vmid, uint16_t *temp, uint16_t *tempint, uint16_t *vaux)
{
    uint8_t msgBuf[14];

    // Read remote temperatures
    msgBuf[0] = addr;
    msgBuf[1] = 0x04;
    i2c_transmit(msgBuf, 2, 1);
    while(i2c_busy());

    msgBuf[0] = addr | 0x01;
    i2c_transmit(msgBuf, 13, 0);
	while(i2c_busy());
	i2c_receive(msgBuf, 13);

    *vtop = ((msgBuf[11] & 0x3F) * 256) + msgBuf[12] + 8192;  // Add 2.5V offset (8192) - see 2990 datasheet p17
    if(msgBuf[3] & 0x40)
    {
        *vmid = 0x0000;  // If less than zero, set to zero
    }
    else
    {
        *vmid = ((msgBuf[3] & 0x3F) * 256) + msgBuf[4];
    }
    *temp = (!(msgBuf[7] & 0x60)) ? (((msgBuf[7] & 0x1F) * 256) + msgBuf[8]) : 0;
    *tempint = ((msgBuf[1] & 0x1F) * 256) + msgBuf[2];
    *vaux = ((msgBuf[5] & 0x3F) * 256) + msgBuf[6];

    return (((msgBuf[7] & 0x60) >> 5) & 0x03);  // Return short/open status bits
}

uint8_t decInHex(uint16_t temp)
{
    uint8_t tens=0, ones=0;
//    uint8_t t = ((temp - ((273*16)+2)) >> 4) & 0xFF;
    uint8_t t = temp;
    tens = (t / 10) & 0x0F;
	ones = (t - tens * 10) & 0x0F;
	return ((tens << 4) & 0xF0) + ones;
}

uint16_t offsetGainAdjust(uint16_t t)
{
    return ( ((uint32_t)tempcal_gain * (uint32_t)((tempcal_offset & 0x80) ? (t - (tempcal_offset & 0x7F)) : (t + tempcal_offset))) / 0x8000);  // See p10 of LTC2990 datasheet
}
