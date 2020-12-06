/*

  Servo array driver. Can make servo pwm signals on any/all pins of a port.
  Consumes one timer event slot.

  Copyright (c) 2020 Thomas Kremer

*/

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 or 3 as
 * published by the Free Software Foundation.
 */

#ifndef __SERVO_ARRAY_H__
#define __SERVO_ARRAY_H__


#include <events.h>

// each context can handle one port.
typedef struct {
  volatile uint8_t* port; // the port register.
  uint16_t pwm[8]; // pulse width in µs. (0-20000; typ. 1000-2000 or 0 for off)
  // internal:
  uint8_t order[8]; // order of pins to switch off
  uint8_t pos,mask;
  // pos combines a 0-8 counter and flags for ddr-setup (0x10),
  //   reordering (0x20), remasking (0x40) and stopping (0x80).
  // mask just caches the total bitmask for the pulse start and DDR setting.
} servo_array_t;

// called between cycles. If glitch-free operation is desired, pwms should only
// be changed in this callback (and announced via _pwms_changed()).
void EVENT_servo_array_idle(servo_array_t* ctx);

// ctx is the servo array context.
// port is &PORTB/&PORTC/&PORTD...
// mask gives the mask of pins to be driven on that port.
// All pwms for the selected pins are initialized to 1500 µs.
void servo_array_init(servo_array_t* ctx, volatile uint8_t* port, uint8_t mask) {
  ctx->port = port;
  ctx->mask = mask;
  ctx->pos = 0;
  uint8_t j = 0;
  for (uint8_t i = 0; i < 8; i++) {
    ctx->pwm[i] = mask&1 ? 1500 : 0;
    ctx->order[j] = i;
    j += mask&1;
    mask >>= 1;
  }
  for (;j < 8; j++)
    ctx->order[j] = 0xff;
}

// call this if you changed channels from 0 to nonzero
static inline void servo_array_channels_added(servo_array_t* ctx) {
  ctx->pos |= 0x70;
}

// you can call this to make sure, changed pwms take effect immediately.
// Otherwise there may be one more pulse with a width between old and new value.
static inline void servo_array_pwms_changed(servo_array_t* ctx) {
  ctx->pos |= 0x20;
}

// change the pwm of a channel. It will take effect at latest 2 pulses from now.
// If you "add" a pin by changing it from 0 to nonzero, you need to call
//   servo_array_channels_added afterwards.
// Setting a pin to 0 automatically removes the pin (it will still be
//   set to low eventually though).
static inline void servo_array_setpwm(servo_array_t* ctx, uint8_t pin, uint16_t pwm) {
  ctx->pwm[pin] = pwm;
}

// simple getter.
static inline uint16_t servo_array_getpwm(servo_array_t* ctx, uint8_t pin) {
  return ctx->pwm[pin];
}

// internal: recalculate the mask and initialize the order array
void servo_array_remask(servo_array_t* ctx) {
  uint8_t mask = 0;
  uint8_t j = 0;
  for (uint8_t i = 0; i < 8; i++) {
    mask >>= 1;
    if (ctx->pwm[i] != 0) {
      mask |= 0x80;
      ctx->order[j] = i;
      j++;
    }
  }
  for (;j < 8; j++)
    ctx->order[j] = 0xff;
  ctx->mask = mask;
}

// internal: sort the order array
void servo_array_reorder(servo_array_t* ctx) {
  // sort (insertion sort, because it is simple, stable,
  //  optimal for mostly-presorted lists and N<=8 is small)
  for (uint8_t l = 0; l < 8; l++) {
    uint8_t ch = ctx->order[l];
    if (ch == 0xff)
      break;
    uint16_t val = ctx->pwm[ch];
    uint8_t i = l;
    while (i != 0) {
      uint16_t val2 = ctx->pwm[ctx->order[i-1]];
      if (val2 > val) {
        ctx->order[i] = ctx->order[i-1];
        i--;
      } else {
        break;
      }
    }
    ctx->order[i] = l;
  }
}
//void inttohex(uint32_t value, char* dest, int count);

// TODO: should we have a more absolute approach to time?
// TODO: event "idle", stopping and reordering should already happen at the end of the last cycle rather than at the beginning of the new one.
// timer event handler.
void servo_array_ontimer(void* param) {
  servo_array_t* ctx = (servo_array_t*) param;
  uint16_t time_us = 0;
  uint8_t pos = ctx->pos & 0x0f;
  
  uint8_t ch = pos < 8 ? ctx->order[pos] : 0xff;
  uint16_t t0 = ch != 0xff ? ctx->pwm[ch] : 0;

  //if (ctx->port == &PORTC) {
  //  LEDs_ToggleLEDs(LEDS_LED1);
  //}
  //LEDs_TurnOnLEDs(LEDS_LED1);
  //  if (ctx->port == &PORTC && ch2 == 0)
  //    LEDs_TurnOffLEDs(LEDS_LED1);
  if (ch == 0xff) {
    EVENT_servo_array_idle(ctx); // if you want to change pwms, now is the time.
    if (ctx->pos & 0x80) {
      // stop flag. Also indicates, there is still a handler queued.
      ctx->pos = 0;
      return;
    }
    if (ctx->pos & 0x40) {
      // use the old mask to low anything left.
      *ctx->port &= ~ctx->mask;
      servo_array_remask(ctx);
    }
    if (ctx->pos & 0x20)
      servo_array_reorder(ctx);
    if (ctx->pos & 0x10)
      *(ctx->port-1) |= ctx->mask;
    *ctx->port |= ctx->mask;
    pos = 0;
    //time_us = ctx->pwm[ctx->order[0]];
    //if (ctx->port == &PORTC) {
    //  LEDs_TurnOnLEDs(LEDS_LED1);
    //}
  } else {
    *ctx->port &= ~(1<<ch);
    pos++;
    //if (ctx->port == &PORTC && ch == 0)
    //  LEDs_TurnOffLEDs(LEDS_LED1);
  }
  while (1) {
    uint8_t ch2 = pos < 8 ? ctx->order[pos] : 0xff;
    uint16_t t1 = ch2 != 0xff ? ctx->pwm[ch2] : 20000;
    if (t1 > t0) {     // Default case: next event is in the future
      time_us = t1-t0; // schedule by time difference
      break;
    }
    if (ch2 == 0xff) { // some pwm was set above 20000 µs.
      time_us = 0;     // Restart next cycle immediately.
      break;
    }
    if (t1 < t0) {
      // we found a non-sorted list. Mark for re-sorting.
      // set the reorder flag and possibly the remask flag.
      uint8_t flag = t1 == 0 ? 0x60 : 0x20;
      ctx->pos |= flag;
      // This pin's pulse will be between its old pwm and its new pwm.
      // That shouldn't be a problem for servos and other slow analog devices.
    }
    *ctx->port &= ~(1<<ch2); // The next channel is already due. Continue.
    pos++;
    //if (ctx->port == &PORTC && ch2 == 0)
    //  LEDs_TurnOffLEDs(LEDS_LED1);
  }
  ctx->pos = pos | (ctx->pos & 0xf0);

  uint32_t dt = (((int32_t)time_us)*usec2ticks(1,TIMER_DIV));
  /*
  if (ctx->port == &PORTC) {
    char msg[9];
    inttohex(dt,&msg[0],8);
    msg[8] = '\n';
    usart_write(msg,9);
  }
  */
  enqueue_event_rel(dt,
      //(((int32_t)time_us)*usec2ticks(1<<8,TIMER_DIV)) >> 8,
                    &servo_array_ontimer,ctx);
}

// starts the servo array. It will automatically determine the mask of used
// pins and set their DDR bits to output low.
void servo_array_start(servo_array_t* ctx) {
  // all work is done in the first ontimer event.
  // servo_array_remask(ctx);
  //servo_array_reorder(ctx);
  //ctx->pos = 8;
  //*ctx->port |= ctx->mask;
  if (ctx->pos & 0x80) {
    // There is already an event handler queued.
    ctx->pos &= 0x7f;
  } else {
    //servo_array_remask(ctx);
    //*ctx->port &= ~ctx->mask;
    //*(ctx->port-1) |= ctx->mask;
    ctx->pos = 0x7f;
    servo_array_ontimer(ctx);
  }
  //enqueue_event_rel(0,&servo_array_ontimer,ctx);
}

// stop gracefully; if we just shut off the signal, the servos would get
//   a short pulse directing them downward.
static inline void servo_array_stop(servo_array_t* ctx) {
  ctx->pos |= 0x80;
}

// shut it all down. NOW. output pins stay where they happen to be.
void servo_array_kill_all() {
  dequeue_events(&servo_array_ontimer);
}

#endif
