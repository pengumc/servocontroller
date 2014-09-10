// name: servocontrollerI2C.c
// Author: Michiel van der Coelen
// contact: Michiel.van.der.coelen@gmail.com
// date: 2014-09
// tabsize: 2
// width: 80
// This code is distributed under the GNU Public License
//    which can be found at http:  // www.gnu.org/licenses/gpl.txt


// MCU = atmega32
// Lfuse EE
// 1 1 1 0 0 0 0 1 : 0xE1 (default)
// 1 1 1 0 1 1 1 0 : 0xEE
// | | | | | | | |
// | | | | | | | +-- CKSEL0 : clock select 1110 = max freq external crystal
// | | | | | | +-- CKSEL1
// | | | | | +-- CKSEL2
// | | | | +-- CKSEL3
// | | | +-- SUT0
// | | +-- SUT1
// | +-- BODEN
// +-- BODLEVEL

// Hfuse DA (bootsector size 1024)
// 1 0 0 1 1 0 0 1 : 0x99 (default)
// 1 1 0 1 1 0 1 0 : 0xDA (old)
// 1 1 0 0 1 0 0 1 : 0xC9 (wanted)
// | | | | | | | |
// | | | | | | | +-- BOOTRST
// | | | | | | +-- BOOTSZ0
// | | | | | +-- BOOTSZ1
// | | | | +-- EESAVE
// | | | +-- CKOPT progam (set to 0) to allow high freq
// | | +-- SPIEN
// | +-- JTAGEN
// +-- OCDEN

// F_CPU=12000000 // defined in makefile
// all comments assume this clock frequency, you'll have to recalculate most
// values when you change it
// when changing MCU, timer registers may have different names.

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "i2c_header.h"

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x&(1<<y))
#define TOG(x,y) (x^=(1<<y))

#define SERVO_AMOUNT 12

#define ASM400HI 0x03
#define ASM400LO 0xC0
#define ASM1700LO 0xA0
#define ASM1700HI 0x03
// 1100e-6 * 12e6 / 22 = 600
#define MIDPULSE 600

#define SHORT_INTERVAL 30
#define REMAINING_TIME 115

typedef struct {
  union {
    uint16_t stop;
    uint8_t stopbytes[2];
  } servo;
  uint8_t port;
  uint8_t pin;
} ServoChannel_t;

ServoChannel_t servo_channels[SERVO_AMOUNT];
volatile ServoChannel_t* group_ptr;
uint8_t servo_cycle_counter = 0;

//i2c
uint8_t r_index =0;
uint8_t recv[BUFLEN_SERVO_DATA]; // buffer to store received bytes
uint8_t t_index=0;
uint8_t tran[BUFLEN_SERVO_DATA];
uint8_t new_cmd=0;
uint8_t reset=0;
uint8_t command = 0;

// -----------------------------------------------------------------------------
//                                                                 init_channels
// -----------------------------------------------------------------------------
inline void init_channels() {
  // hardcoded ports and pins
  // use _SFR_IO_ADDR(PORTA) + 32 for ports 
  // and ~(1<<pinnumber) for pins
  // servo.stop = base value - (x * 12MHz) / 22
  //    where x is the time you want added to 400us

  uint8_t i = 0;
  for (i = 0; i < 8; ++i) {
    servo_channels[i].servo.stop = ((ASM1700HI<<8)|(ASM1700LO)) - MIDPULSE;
    servo_channels[i].port = _SFR_IO_ADDR(PORTA) + 32;
    servo_channels[i].pin = ~(1<<i);
  }
  for (i = 8; i < SERVO_AMOUNT; ++i) {
    servo_channels[i].servo.stop = ((ASM1700HI<<8)|(ASM1700LO)) - MIDPULSE;
    servo_channels[i].port = _SFR_IO_ADDR(PORTC) + 32;
    servo_channels[i].pin = ~(1<<(i - 6));
  }
  DDRA = 0xFF;
  PORTA = 0x00;
  DDRC |= (1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);
  PORTC &= ~((1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5));

  servo_channels[0].servo.stop = ((ASM1700HI<<8)|(ASM1700LO)) - 328;
  

  // TODO add offsets per pin based on their pos in cycle
}


// -----------------------------------------------------------------------------
//                                                                      init_I2C
// -----------------------------------------------------------------------------
inline void init_I2C(){
  // load slave address
  TWAR = (0x01<<1); // we're using address 0x01 
  // enable I2C hardware
  TWCR = (1<<TWEN)|(1<<TWEA);
}


// -----------------------------------------------------------------------------
//                                                                   init_timers
// -----------------------------------------------------------------------------
void init_timers() {
  // using timer 0 (8 bit) with 1024 prescaler in CTC mode
  // to get about 50 Hz
  TCCR0 = (1<<WGM01)|(1<<CS02)|(1<<CS00);
  OCR0 = SHORT_INTERVAL;
  SET(TIMSK, OCIE0);
  sei();
}

// -----------------------------------------------------------------------------
//                                                                     heartbeat
// -----------------------------------------------------------------------------
inline void heartbeat() {
  /*
   * 0  15 30  45 250
   * 1  0  1   0  0   
   */
  if (servo_cycle_counter < 15) {
    SET(PORTD, PD3);
  } else if (servo_cycle_counter < 30) {
    CLR(PORTD, PD3);
  } else if (servo_cycle_counter < 45) {
    SET(PORTD, PD3);
  } else if (servo_cycle_counter < 100) {
    CLR(PORTD, PD3);
  } else {
    servo_cycle_counter = 255;
  }
}

// -----------------------------------------------------------------------------
//                                                                    handle_I2C
// -----------------------------------------------------------------------------
inline void handle_I2C(){ // slave version
  // stop values for servos are sent lsb first

  // check if we need to do any software actions
  if (CHK(TWCR,TWINT)) {
    SET(PORTD, PD5);
    switch(TW_STATUS){
// --------------Slave receiver------------------------------------
    // SLA_W received and acked, prepare for data receiving
    case 0x60:  
      TWACK;
      r_index =0;
      break;
    case 0x80:  // a byte was received, store it and 
                // setup the buffer to recieve another
      // check if the received buffer is a command
      if (r_index ==0) {
        if (TWDR > ASM1700HI) {
          new_cmd = TWDR;
        } else {
          recv[r_index] = TWDR;
          new_cmd =0;
        }
      } 
      if (r_index>0) {
        if (new_cmd == 0) {
          recv[r_index]= TWDR;
        }
        if (new_cmd != TWDR) new_cmd = 0;
      }
      r_index++;
      // don't ack next data if buffer is full
      if (r_index >= BUFLEN_SERVO_DATA) {
        TWNACK;
        command = new_cmd;
      } else {
    TWACK;
   }
   break;
    case 0x68:  // adressed as slave while in master mode.
              // should never happen, better reset;
      reset=1;
    case 0xA0: // Stop or rep start, reset state machine
      TWACK;
      break;
// -------------- error recovery ----------------------------------
    case 0x88: // data received  but not acked
      // should not happen if the master is behaving as expected
      // switch to not adressed mode
      TWACK;
      break;
// ---------------Slave Transmitter--------------------------------
    case 0xA8:  // SLA R received, prep for transmission
                // and load first data
      t_index=1;
      TWDR = servo_channels[0].servo.stopbytes[0];
      TWACK;
      break;
    case 0xB8:  // data transmitted and acked by master, load next
      TWDR = servo_channels[t_index>>1].servo.stopbytes[1-t_index%2];
      t_index++;
      // designate last byte if we're at the end of the buffer
      if(t_index >= BUFLEN_SERVO_DATA) TWNACK;
      else TWACK;
      break;
    case 0xC8: // last byte send and acked by master
    // last bytes should not be acked, ignore till start/stop
      // reset=1;
    case 0xC0: // last byte send and nacked by master 
    // (as should be)
      TWACK;
      break;
// --------------------- bus error---------------------------------
    // illegal start or stop received, reset the I2C hardware
    case 0x00: 
      TWRESET;
      break;
    }
  } else {
    CLR(PORTD, PD5);
  }
}


// =============================================================================
//                                                                          main
// =============================================================================
int main() {
  DDRD = (1<<PD5)|(1<<PD3);
  PORTD = (0<<PD5)|(0<<PD3);  // red , green
  init_channels();
  init_timers();
  group_ptr = &servo_channels[0];
  init_I2C();
  wdt_enable(WDTO_1S);
  while(1) {
    if (!reset) {
      wdt_reset();
      heartbeat();
      handle_I2C();
      if (r_index == BUFLEN_SERVO_DATA) {
        uint8_t i = 0;
        for (i = 0; i < BUFLEN_SERVO_DATA; ++i) {
          servo_channels[i>>1].servo.stopbytes[1-i%2] = recv[i];
        }
      }
    } else { 
      SET(PORTD, PD5);
    }
  }
}

// -----------------------------------------------------------------------------
//                                                                       TIMER 0
// -----------------------------------------------------------------------------
ISR(TIMER0_COMP_vect){
  /* checkloop
  counter == stop
    3 first cp skips
    1 second cp
    1 brne, but r16 = r24 so don't branch
    7 clear pin
    3 jump somewhere
   ---
    15

  counter != stop, r17 = r25 but r16 != r24
    3 first cp skips
    1 cp
    2 brne true, so branch to noclear2
   ---
    6 still needs 9 nops

  count  != stop, r17 != r25
   1 fist cp doesn't skip
   3 jump to noclear1
  ---
   4 still needs 11 nops

   total loopcycles = 10 (2 for sbiw, 2 for branch)
   add 5 cycles for setting port low once


   2 port address 1
   3 pin 1
   4 port address 2
   5 pin 2
   6 port address 3
   7 pin 3
   8 value read at pin
   9 temp

   24 counter value
   25 counter value
   16 stop 1
   17 stop 1
   18 stop 2
   19 stop 2
   20 stop 3
   20 stop 3

   counter = stop only occurs once, no need to adjust rest to it, just substract
   difference from stop value once.
  */
  __asm__(
          "CLI \n\t"
          // set all ports of group high
          // first channel
          // load pin address on Y
          "LDD r28, %a4+2 \n\t"
          "MOV r29, __zero_reg__ \n\t"
          "MOV r2, r28 \n\t" // also store pin addr in r2
          // read pin value
          "LD r8, Y \n\t"
          // load proper pin value from channel.pin
          "LDD r3, %a4+3 \n\t"
          // create new value for port
          "LDI r24, 0xFF \n\t"
          "EOR r24, r3 \n\t" // invert r3
          "OR r24, r8 \n\t"
          // write to port
          "ST Y, r24 \n\t"

          // repeat for second and third channel
          "LDD r28, %a4+6 \n\t"
          "MOV r4, r28 \n\t"
          "LD r8, Y \n\t"       
          "LDD r5, %a4+7 \n\t"  
          "LDI r24, 0xFF \n\t"  
          "EOR r24, r5 \n\t"    
          "OR r24, r8 \n\t"     
          "ST Y, r24 \n\t"

          "LDD r28, %a4+10 \n\t"
          "MOV r6, r28 \n\t"
          "LD r8, Y \n\t"
          "LDD r7, %a4+11 \n\t"
          "LDI r24, 0xFF \n\t"
          "EOR r24, r7 \n\t"
          "OR r24, r8 \n\t"
          "ST Y, r24 \n\t"

          // wait 400 us
          "LDI r25, %0 \n\t"
          "LDI r24, %1 \n\t"
         "loopje: \n\t"
          "WDR \n\t"
          "SBIW r24, 0x01 \n\t"
          "BRNE loopje \n\t"
          // load counter value for 1700 us
          "LDI r25, %2 \n\t"
          "LDI r24, %3 \n\t"
          "LD r16, %a4 \n\t"   // 1
          "LDD r17, %a4+1 \n\t"// 1
          "LDD r18, %a4+4 \n\t"   // 1
          "LDD r19, %a4+5 \n\t"// 1
          "LDD r20, %a4+8 \n\t"   // 1
          "LDD r21, %a4+9 \n\t"// 1

         "checkloop: \n\t"
          // first in group
          "CPSE r17, r25 \n\t"  
          "JMP noclear1A \n\t"
          "CP r16, r24 \n\t"
          "BRNE noclear1B \n\t"
          // read value from pin
          "MOV r28, r2 \n\t"
          "LD r8, Y \n\t"
          // create new port value
          "MOV r9, r3 \n\t"
          "AND r9, r8 \n\t"
          // write to port
          "ST Y, r9 \n\t"
          "JMP finalpart1 \n\t"

         "noclear1A: \n\t"
          "nop \n\t"
          "nop \n\t"
         "noclear1B: \n\t"
         "finalpart1: \n\t"  

          //second in group
          "CPSE r19, r25 \n\t"
          "JMP noclear2A \n\t"
          "CP r18, r24 \n\t"
          "BRNE noclear2B \n\t"
          "MOV r28, r4 \n\t"
          "LD r8, Y \n\t"
          "MOV r9, r5 \n\t"
          "AND r9, r8 \n\t"
          "ST Y, r9 \n\t"
          "JMP finalpart2 \n\t"
         "noclear2A: \n\t"
          "nop \n\t"
          "nop \n\t"
         "noclear2B: \n\t"
          "finalpart2: \n\t"
          //third
          "CPSE r21, r25 \n\t"
          "JMP noclear3A \n\t"
          "CP r20, r24 \n\t"
          "BRNE noclear3B \n\t"
          "MOV r28, r6 \n\t"
          "LD r8, Y \n\t"  
          "MOV r9, r7 \n\t"
          "AND r9, r8 \n\t"
          "ST Y, r9 \n\t"
          "JMP finalpart3 \n\t"
         "noclear3A: \n\t"
          "nop \n\t"
          "nop \n\t"
         "noclear3B: \n\t"
          "finalpart3: \n\t"

          "SBIW r24, 0x01 \n\t"
          "BRNE checkloop \n\t"
          // clear all at end
          "MOV r28, r2 \n\t"
          "LD r8, Y \n\t" 
          "MOV r9, r3 \n\t" 
          "AND r9, r8 \n\t" 
          "ST Y, r9 \n\t"
          "MOV r28, r4 \n\t"
          "LD r8, Y \n\t"
          "MOV r9, r5 \n\t"
          "AND r9, r8 \n\t"
          "ST Y, r9 \n\t"
          "MOV r28, r6 \n\t"
          "LD r8, Y \n\t"
          "MOV r9, r7 \n\t"
          "AND r9, r8 \n\t" // TODO(michie): no need for r9 really
          "ST Y, r9 \n\t"
          
          "SEI \n\t"
  ::"M"(ASM400HI),"M"(ASM400LO),
    "M"(ASM1700HI),"M"(ASM1700LO),
    "z"(group_ptr):"r2","r3","r4","r5","r6","r7","r8","r9","r16","r17","r18",
    "r19","20","r21","r24","r25","r28","r29");
  
  // point to the next 3 channels for the next timer interrupt
  if (group_ptr == &servo_channels[0]) {
    group_ptr = &servo_channels[3];
    OCR0 = SHORT_INTERVAL;
  } else if (group_ptr == &servo_channels[3]) {
    group_ptr = &servo_channels[6];
  } else if (group_ptr == &servo_channels[6]) {
    group_ptr = &servo_channels[9];
  } else if (group_ptr == &servo_channels[9]) {
    group_ptr = &servo_channels[0];
    OCR0 = REMAINING_TIME;
    ++servo_cycle_counter;
  }
}
