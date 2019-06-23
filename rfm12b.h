/*

  RFM12 library - secondary module (for use in a two-modules test setting)

  Copyright (c) 2016 Thomas Kremer

*/

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 or 3 as
 * published by the Free Software Foundation.
 */

/********************************************************************
secondary RFM12 module (rfm12b_* namespace), wired to:

B2 <-> SDO
B1 <-> nIRQ
B4 <-> FSK/DATA/nFFS

C2 <-> nSEL
C0 <-> SCK
B5 <-> SDI

left to connect:
  VCC,GND (VDI?,nRES?)

**********************************************************************/

#define RFXX_PORT_SEL PORTC
#define RFXX_PIN_SEL PINC
#define RFXX_DDR_SEL DDRC
#define RFXX_SEL 2

#define RFXX_PORT_SDI PORTB
#define RFXX_PIN_SDI PINB
#define RFXX_DDR_SDI DDRB
#define RFXX_SDI 5

#define RFXX_PORT_SCK PORTC
#define RFXX_PIN_SCK PINC
#define RFXX_DDR_SCK DDRC
#define RFXX_SCK 0

#define RFXX_PORT_SDO PORTB
#define RFXX_PIN_SDO PINB
#define RFXX_DDR_SDO DDRB
#define RFXX_SDO 2

#define RFXX_PORT_DATA PORTB
#define RFXX_PIN_DATA PINB
#define RFXX_DDR_DATA DDRB
#define RFXX_DATA 4

#define RFXX_PORT_NIRQ PORTB
#define RFXX_PIN_NIRQ PINB
#define RFXX_DDR_NIRQ DDRB
#define RFXX_NIRQ 1

// The IRQ-pins share a common pin-change interrupt PCI0:
//#define RFXX_NIRQ_ISR PCINT1
//#define RFXX_NIRQ_ISR PCINT3


#define SUFFIX(x,y) x##b##y

#include <rfm12_internal.h>

