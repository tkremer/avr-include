/*

  RFM12 library - internal implementation

  Copyright (c) 2016-2019 Thomas Kremer

*/

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 or 3 as
 * published by the Free Software Foundation.
 */


/********************************************************************
generic rfm12 module. All pins can be configured with #defines, multiple
 modules require multiple inclusions of this file with different SUFFIXes
 defined.


due to space constraints, the default wirings were the following:

B2 <-> SDO
B3 <-> nIRQ
B4 <-> FSK/DATA/nFFS  (must be set to logic-high)

C1 <-> nSEL
C0 <-> SCK
B5 <-> SDI

which already gives us all neccessary logic pins.
left to connect:
  VCC,GND (VDI?,nRES?)

**********************************************************************/

#include <avr/io.h>
#ifndef RFXX_SEL
#define RFXX_PORT_SEL PORTC
#define RFXX_PIN_SEL PINC
#define RFXX_DDR_SEL DDRC
#define RFXX_SEL 1
#endif

#ifndef RFXX_SDI
#define RFXX_PORT_SDI PORTB
#define RFXX_PIN_SDI PINB
#define RFXX_DDR_SDI DDRB
#define RFXX_SDI 5
#endif

#ifndef RFXX_SCK
#define RFXX_PORT_SCK PORTC
#define RFXX_PIN_SCK PINC
#define RFXX_DDR_SCK DDRC
#define RFXX_SCK 0
#endif

#ifndef RFXX_SDO
#define RFXX_PORT_SDO PORTB
#define RFXX_PIN_SDO PINB
#define RFXX_DDR_SDO DDRB
#define RFXX_SDO 2
#endif

#ifndef RFXX_DATA
#define RFXX_PORT_DATA PORTB
#define RFXX_PIN_DATA PINB
#define RFXX_DDR_DATA DDRB
#define RFXX_DATA 4
#endif

#ifndef RFXX_NIRQ
#define RFXX_PORT_NIRQ PORTB
#define RFXX_PIN_NIRQ PINB
#define RFXX_DDR_NIRQ DDRB
#define RFXX_NIRQ 3
#endif

#define RFXX_OUTPUT(pin) RFXX_DDR_##pin  |= (1<<RFXX_##pin)
#define RFXX_INPUT(pin)  RFXX_DDR_##pin  &= ~(1<<RFXX_##pin)
#define RFXX_HI(pin)     RFXX_PORT_##pin |= (1<<RFXX_##pin)
#define RFXX_LOW(pin)    RFXX_PORT_##pin &= ~(1<<RFXX_##pin)
#define RFXX_GET(pin)  ((RFXX_PIN_##pin  & (1<<RFXX_##pin)) != 0)
#define RFXX_TOGGLE(pin) RFXX_PIN_##pin  = (1<<RFXX_##pin)


#ifndef SUFFIX
#define SUFFIX(x,y) x##y
#endif
#define S(x) SUFFIX(x,)
//#define P(x) rfm12##SUFFIX##x
#define P(x) SUFFIX(rfm12,_##x)

#define rfm12_cmd P(cmd)
#define rfm12_txfifo_ready P(txfifo_ready)
#define rfm12_wait_txfifo_ready P(wait_txfifo_ready)
#define rfm12_init_pins P(init_pins)
#define rfm12_init P(init)

#define rfm12_status P(status)
#define rfm12_handle_interrupt P(handle_interrupt)
#define rfm12_clear_interrupts P(clear_interrupts)
#define rfm12_listen P(listen)
#define rfm12_listen_reset P(listen_reset)

#define rfm12_state P(state)
#define rfm12_set_state P(set_state)
#define rfm12_read_handler P(read_handler)
#define rfm12_write_handler P(write_handler)
#define rfm12_send_packet P(send_packet)
#define rfm12_stop_sending P(stop_sending)
#define rfm12_stop_receiving P(stop_receiving)
#define rfm12_poll_wait P(poll_wait)

#ifndef RFXX_DEFS
  #define RFXX_DEFS
  #define RFXX_FLAG_FFIT (1<<15) // rxfifo ready (contains data) (er=1)
  #define RFXX_FLAG_RGIT (1<<15) // txfifo ready (has free space) (er=0)
  #define RFXX_FLAG_POR  (1<<14) // power-on reset
  #define RFXX_FLAG_FFOV (1<<13) // rxfifo overflow (er=1)
  #define RFXX_FLAG_RGUR (1<<13) // txfifo underrun (er=0)
  #define RFXX_FLAG_WKUP (1<<12) // wakeup-timer overflow
  #define RFXX_FLAG_EXT  (1<<11) // external interrupt (nINT low)
  #define RFXX_FLAG_LBD  (1<<10) // low battery detect
  #define RFXX_FLAG_FFEM (1<< 9) // fifo empty
  #define RFXX_FLAG_RSSI (1<< 8) // strong incoming signal detected (er=1)
  #define RFXX_FLAG_ATS  (1<< 8) // strong RF signal detected (er=0)
  #define RFXX_FLAG_DQD  (1<< 7) // data quality detector output(?)
  #define RFXX_FLAG_CRL  (1<< 6) // clock recovery locked
  #define RFXX_FLAG_ATGL (1<< 5) // toggles every AFC cycle
  #define RFXX_FLAG_OFFS6 (1<< 4) // frequency offset (5-bit)
  #define RFXX_FLAG_OFFS3 (1<< 3)
  #define RFXX_FLAG_OFFS2 (1<< 2)
  #define RFXX_FLAG_OFFS1 (1<< 1)
  #define RFXX_FLAG_OFFS0 (1<< 0)
  #define RFXX_TRANSIENT_FLAGS (RFXX_FLAG_POR | RFXX_FLAG_FFOV | RFXX_FLAG_WKUP | RFXX_FLAG_EXT)


// We need a state machine for the transceiver:
  #define RFXX_STATEFLAG_LISTEN 0x80
  #define RFXX_STATEFLAGS 0x80
  #define RFXX_STATE_IDLE 0
  #define RFXX_STATE_LISTEN 1
  #define RFXX_STATE_RX 2
  #define RFXX_STATE_TX_SYNC0 3
  #define RFXX_STATE_TX_SYNC1 4
  #define RFXX_STATE_TX 5
  #define RFXX_STATE_TX_END 6
  #define RFXX_STATE_TX_RESYNC 7

  #define RFXX_EV_CONT  0
  #define RFXX_EV_START 1
  #define RFXX_EV_END   2
  #define RFXX_EV_TX_END 0x100
  #define RFXX_EV_TX_RESYNC 0x101

// IDLE -> LISTEN -> RX -> IDLE -> LISTEN -> ... -> IDLE
// IDLE -> TX_SYNC0 -> TX_SYNC1 -> TX -> TX_END -> IDLE


// freq f = 430+F/400 MHz, F = 0x640 in 0xA640 command.
// => f = 430+1600/400 MHz = 434 MHz
// data rate r = 10000/29/(R+1)/(1+cs*7) kbps, R+(cs<<7)=0xXX in 0xC6XX command.
// r \in [336..172k] bps
const uint16_t rfm12_init_seq[] PROGMEM = {
  0x80D7, //EL,EF,12.0pF
  // example init() actually was a jammer (always-on):
  //0x8239, //!er,!ebb,ET,ES,EX,!eb,!ew,DC
  0x8201, //!er,!ebb,!ET,!ES,!EX,!eb,!ew,DC
  0xA640, // 434 MHz; A140=430.8MHz
  0xC647, //4.8kbps
  0x94A0, //VDI,FAST,134kHz,0dBm,-103dBm
  0xC2AC, //AL,!ml,DIG,DQD4
  0xCA81, //FIFO8,SYNC,!ff,DR
  0xC483, //@PWR,NO RSTRIC,!st,!fi,OE,EN
  // frequency modulation bandwidth:
  // f_out = f0+(-1)^sign*(m+1)*15kHz; delta_f =(5+1)*15kHz = 90kHz:
  0x9850, //!mp,9810=30kHz,MAX OUT
  0xE000, //NOT USE
  0xC800, //NOT USE
  0xC400, //1.66MHz,2.2V
};
#define rfm12_init_seq_len (sizeof(rfm12_init_seq)/sizeof(rfm12_init_seq[0]))

#endif

uint16_t rfm12_status = 0;
uint8_t rfm12_state = RFXX_STATE_IDLE;

void rfm12_init_pins() {
  RFXX_INPUT(NIRQ);
  RFXX_INPUT(SDO);
  RFXX_HI(SEL);
  RFXX_HI(SDI);
  RFXX_LOW(SCK);
  RFXX_HI(DATA);
  RFXX_OUTPUT(SEL);
  RFXX_OUTPUT(SDI);
  RFXX_OUTPUT(SCK);
  RFXX_OUTPUT(DATA);
}

uint16_t rfm12_cmd(uint16_t cmd) {
  uint16_t res = 0;
  RFXX_LOW(SCK);
  RFXX_LOW(SEL);
  for (int i = 0; i < 16; i++) {
    if (cmd & 0x8000) {
      RFXX_HI(SDI);
    } else {
      RFXX_LOW(SDI);
    }
    RFXX_TOGGLE(SCK);
    cmd <<= 1;
    res <<= 1;
    if (RFXX_GET(SDO)) res |= 1;
    RFXX_TOGGLE(SCK);
  }
  RFXX_HI(SEL);
  return res;
}


void rfm12_init() {
  rfm12_init_pins();
  for (int i = 0; i < rfm12_init_seq_len; i++) {
    rfm12_cmd(pgm_read_word(&rfm12_init_seq[i]));
  }
  rfm12_status = 0;
  rfm12_state = RFXX_STATE_IDLE;
}

bool rfm12_set_state(uint8_t state) {
  uint8_t rawstate = rfm12_state;
  uint8_t oldstate = rawstate & ~RFXX_STATEFLAGS;
  if (oldstate != state) {
    switch(state) {
      case RFXX_STATE_IDLE:
        rfm12_cmd(0x8201);//!er,!ebb,!ET,!ES,!EX,!eb,!ew,DC
        rfm12_cmd(0xCA81);
        break;
      case RFXX_STATE_LISTEN:
        if (oldstate == RFXX_STATE_RX) {
          rfm12_cmd(0xCA81); // "The FIFO fill stops when this bit is cleared."
        } else { // if (oldstate == RFXX_STATE_IDLE) {
          rfm12_cmd(0x82C9);//ER,ebb,!ET,!ES,EX,!eb,!ew,DC
        } //else return 0;
        rfm12_cmd(0xCA83); // "FIFO fill will be enabled after sync. pattern reception."
        break;
      case RFXX_STATE_RX:
        if (oldstate != RFXX_STATE_LISTEN)
          return 0;
        break;
      case RFXX_STATE_TX_SYNC0:
        if (oldstate != RFXX_STATE_IDLE &&
            oldstate != RFXX_STATE_LISTEN &&
            oldstate != RFXX_STATE_TX_RESYNC)
          return 0;
        rfm12_cmd(0xB800 | 0xaa);
        if (oldstate != RFXX_STATE_TX_RESYNC) {
          rfm12_cmd(0xB800 | 0xaa);
          rfm12_cmd(0x8239);//!er,!ebb,ET,ES,EX,!eb,!ew,DC
        }
        break;
      case RFXX_STATE_TX_SYNC1:
        if (oldstate != RFXX_STATE_TX_SYNC0)
          return 0;
        rfm12_cmd(0xB800 | 0x2d);
        break;
      case RFXX_STATE_TX:
        if (oldstate != RFXX_STATE_TX_SYNC1)
          return 0;
        rfm12_cmd(0xB800 | 0xd4);
        break;
      case RFXX_STATE_TX_END:
      case RFXX_STATE_TX_RESYNC:
        if (oldstate != RFXX_STATE_TX)
          return 0;
        rfm12_cmd(0xB800 | 0xaa);
          // need to write the dummy byte before going idle.
        break;
      default:
        return 0;
    }
    rfm12_state = state | (rawstate & RFXX_STATEFLAGS);
    //rfm12_handle_interrupt();
  }
  return 1;
}

// called for every packet read event.
// event: RFXX_EV_START, _EV_END or _EV_CONT.
// data: received byte, invalid for _EV_END.
bool rfm12_read_handler(uint8_t event, uint8_t data);

// called for every packet send event.
// should return a byte to send or RFXX_EV_TX_END for end-of-transfer.
//  or RFXX_EV_TX_RESNYC for restart-transfer.
uint16_t rfm12_write_handler();

void rfm12_handle_interrupt() {
  while (!RFXX_GET(NIRQ)) {
    uint16_t val = rfm12_cmd(0x0000);
    val = (rfm12_status & RFXX_TRANSIENT_FLAGS) | val;
    rfm12_status = val;

    uint8_t rawstate = rfm12_state;
    uint8_t state = rawstate & ~RFXX_STATEFLAGS;
    uint8_t newstate = state;
    switch (state) {
      case RFXX_STATE_IDLE:
        return; // in idle state we shouldn't listen to interrupts anyway...
        break;
      case RFXX_STATE_LISTEN: {
        if (val & RFXX_FLAG_FFIT) {
          newstate = RFXX_STATE_RX;
          uint16_t fifo_data = rfm12_cmd(0xB000);
          rfm12_read_handler(RFXX_EV_START,fifo_data & 0xff);
        }
        break;
      }
      case RFXX_STATE_RX: {
        if (val & RFXX_FLAG_FFIT) {
          uint16_t fifo_data = rfm12_cmd(0xB000);
          uint8_t event = RFXX_EV_CONT;
          if ((val & RFXX_FLAG_DQD) == 0) {
            newstate = RFXX_STATE_IDLE;
            event = RFXX_EV_END;
          }
          bool cont = rfm12_read_handler(event,fifo_data & 0xff);
          if (!cont)
            newstate = RFXX_STATE_IDLE;
        }
        break;
      }
      case RFXX_STATE_TX_SYNC0:
      case RFXX_STATE_TX_SYNC1:
        if (val & RFXX_FLAG_RGIT)
          newstate++;
        break;
      case RFXX_STATE_TX:
        if (val & RFXX_FLAG_RGIT) {
          // get a new byte to send.
          uint16_t data = rfm12_write_handler();
          if (data & 0x100) {
            if (data == RFXX_EV_TX_END)
              newstate = RFXX_STATE_TX_END;
            else if (data == RFXX_EV_TX_RESYNC)
              newstate = RFXX_STATE_TX_RESYNC;
            // no more bytes -> go to TX_END
          } else {
            rfm12_cmd(0xB800 | (data & 0xff));
          }
        }
        break;
      case RFXX_STATE_TX_END:
        if (val & RFXX_FLAG_RGIT)
          newstate = RFXX_STATE_IDLE;
        break;
      case RFXX_STATE_TX_RESYNC:
        if (val & RFXX_FLAG_RGIT)
          newstate = RFXX_STATE_TX_SYNC0;
        break;
      default:
        newstate = RFXX_STATE_IDLE;
        usart_msg("INVALID STATE\n");
    }
    if (newstate == RFXX_STATE_IDLE && (rfm12_state & RFXX_STATEFLAG_LISTEN))
      newstate = RFXX_STATE_LISTEN;
    if (state != newstate) {
      bool res = rfm12_set_state(newstate);
      if (!res) {
        usart_msg("FAIL\n");
      }
    }
  }
}

bool rfm12_listen(bool enable) {
  bool res = 1;
  if (enable) {
    rfm12_state |= RFXX_STATEFLAG_LISTEN;
    if ((rfm12_state & ~RFXX_STATEFLAGS) == RFXX_STATE_IDLE)
      res = rfm12_set_state(RFXX_STATE_LISTEN);
  } else {
    rfm12_state &= ~RFXX_STATEFLAG_LISTEN;
    if ((rfm12_state & ~RFXX_STATEFLAGS) == RFXX_STATE_LISTEN)
      res = rfm12_set_state(RFXX_STATE_IDLE);
  }
  return res;
}

bool rfm12_send_packet() {
  bool res = rfm12_set_state(RFXX_STATE_TX_SYNC0);
  rfm12_handle_interrupt();
  return res;
}

bool rfm12_stop_sending() {
  uint8_t state = rfm12_state & RFXX_STATEFLAG_LISTEN ? RFXX_STATE_LISTEN : RFXX_STATE_IDLE;
  return rfm12_set_state(state);
}

bool rfm12_stop_receiving() {
  uint8_t rawstate = rfm12_state;
  uint8_t oldstate = rawstate & ~RFXX_STATEFLAGS;
  if (oldstate == RFXX_STATE_RX) {
    uint8_t state = rawstate & RFXX_STATEFLAG_LISTEN ? RFXX_STATE_LISTEN : RFXX_STATE_IDLE;
    return rfm12_set_state(state);
  }
  return 1;
}

void rfm12_clear_interrupts() {
  rfm12_status = 0;
}

void rfm12_poll_wait() {
  while(RFXX_GET(NIRQ));
  rfm12_handle_interrupt();
}

// FIXME: rfm12_interrupted { return !RFXX_GET(NIRQ); }


#undef rfm12_cmd
#undef rfm12_txfifo_ready
#undef rfm12_wait_txfifo_ready
#undef rfm12_init
#undef rfm12_init_pins

#undef rfm12_status
#undef rfm12_handle_interrupt
#undef rfm12_clear_interrupts
#undef rfm12_listen
#undef rfm12_listen_reset

#undef rfm12_state
#undef rfm12_set_state
#undef rfm12_read_handler
#undef rfm12_write_handler
#undef rfm12_send_packet
#undef rfm12_stop_sending
#undef rfm12_stop_receiving
#undef rfm12_poll_wait


#undef S
#undef P

#undef RFXX_OUTPUT
#undef RFXX_INPUT
#undef RFXX_HI
#undef RFXX_LOW
#undef RFXX_GET
#undef RFXX_TOGGLE

#undef RFXX_PORT_SEL
#undef RFXX_PIN_SEL
#undef RFXX_DDR_SEL
#undef RFXX_SEL

#undef RFXX_PORT_SDI
#undef RFXX_PIN_SDI
#undef RFXX_DDR_SDI
#undef RFXX_SDI

#undef RFXX_PORT_SCK
#undef RFXX_PIN_SCK
#undef RFXX_DDR_SCK
#undef RFXX_SCK

#undef RFXX_PORT_SDO
#undef RFXX_PIN_SDO
#undef RFXX_DDR_SDO
#undef RFXX_SDO

#undef RFXX_PORT_DATA
#undef RFXX_PIN_DATA
#undef RFXX_DDR_DATA
#undef RFXX_DATA

#undef RFXX_PORT_NIRQ
#undef RFXX_PIN_NIRQ
#undef RFXX_DDR_NIRQ
#undef RFXX_NIRQ

#undef SUFFIX
