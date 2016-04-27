// name: servocontrollerI2C.c
// Author: Michiel van der Coelen
// contact: Michiel.van.der.coelen@gmail.com
// date: 2015-12
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
#include "smbus_slave/smbus_slave.h"
smbus_mem_t smbus_mem;
uint8_t pin_stash[12];

void smbus_receive_command(uint8_t command) {

}

void smbus_block_write_done() {

}

#define SERVO_AMOUNT 12

#define ASM400HI 0x04
#define ASM400LO 0xB0
#define ASM1700LO 0xA0
#define ASM1700HI 0x03
// 1100e-6 * 12e6 / 22 = 600, aka 0x0258
#define MIDPULSE 600

#define SHORT_INTERVAL 30
#define REMAINING_TIME 115

volatile uint8_t* group_ptr;
volatile uint8_t servo_cycle_counter = 0;

uint8_t reset = 0;

#define STATE_INITIALIZING (0)
#define STATE_RUNNING (1)
#define STATE_ERROR (2)
#define POWER_LED_ON() SET(PORTC, PC6)
#define POWER_LED_OFF() CLR(PORTC, PC6)
#define HEARTBEAT_LED_ON() SET(PORTC, PC7)
#define HEARTBEAT_LED_OFF() CLR(PORTC, PC7)
#define ERROR_LED_ON() SET(PORTB, PB0)
#define ERROR_LED_OFF() CLR(PORTB, PB0)
#define COMM_SYNC_HIGH() SET(PORTB, PB1)
#define COMM_SYNC_LOW() CLR(PORTB, PB1)

// -----------------------------------------------------------------------------
//                                                             eeprom_write_byte
// -----------------------------------------------------------------------------
void eeprom_write_byte(uint8_t address, uint8_t value){
  // wait for access
  while(CHK(EECR, EEWE)){
  ;
  }
  EEAR = address;
  EEDR = value;
  SET(EECR,EEMWE);
  SET(EECR, EEWE);
}

// -----------------------------------------------------------------------------
//                                                              eeprom_read_byte
// -----------------------------------------------------------------------------
uint8_t eeprom_read_byte(uint8_t address){
  // wait for access
  while(CHK(EECR, EEWE)){
  ;
  }
  EEAR = address;
  SET(EECR,EERE);
  return EEDR;

}


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
  for (i = 0; i < 12; ++i) {
    smbus_mem.dev_specific.stopbytes[i*2+0] = 0x47;//eeprom_read_byte((i<<1)+1);
    smbus_mem.dev_specific.stopbytes[i*2+1] = 0x01;//eeprom_read_byte((i<<1)+0);
    smbus_mem.dev_specific.pins[i*2+0] = 0xff;
    // smbus_mem.dev_specific.pins[i*2+1] = 0xff; // not needed
  }
  // all of PORTD
  PORTD = 0x00;
  DDRD = 0xFF;
  smbus_mem.dev_specific.ports[0] = _SFR_IO_ADDR(PORTD) + 32;

  pin_stash[0] = ~(1<<0);
  smbus_mem.dev_specific.ports[2] = _SFR_IO_ADDR(PORTD) + 32;
  pin_stash[1] = ~(1<<1);
  smbus_mem.dev_specific.ports[4] = _SFR_IO_ADDR(PORTD) + 32;
  pin_stash[2] = ~(1<<2);
  smbus_mem.dev_specific.ports[6] = _SFR_IO_ADDR(PORTD) + 32;
  pin_stash[3] = ~(1<<3);
  smbus_mem.dev_specific.ports[8] = _SFR_IO_ADDR(PORTD) + 32;
  pin_stash[4] = ~(1<<4);
  smbus_mem.dev_specific.ports[10] = _SFR_IO_ADDR(PORTD) + 32;
  pin_stash[5] = ~(1<<5);
  smbus_mem.dev_specific.ports[12] = _SFR_IO_ADDR(PORTD) + 32;
  pin_stash[6] = ~(1<<6);
  smbus_mem.dev_specific.ports[14] = _SFR_IO_ADDR(PORTD) + 32;
  pin_stash[7] = 0x7F; // compiler thinks ~(1<<7) is 16 bits signed
  // PORTC 2 to 5
  PORTC &= ~((1<<2) | (1<<3) | (1<<4) | (1<<5));
  DDRC  |=   (1<<2) | (1<<3) | (1<<4) | (1<<5);
  smbus_mem.dev_specific.ports[16] = _SFR_IO_ADDR(PORTC) + 32;
  pin_stash[8] = ~(1<<2);
  smbus_mem.dev_specific.ports[18] = _SFR_IO_ADDR(PORTC) + 32;
  pin_stash[9] = ~(1<<3);
  smbus_mem.dev_specific.ports[20] = _SFR_IO_ADDR(PORTC) + 32;
  pin_stash[10] = ~(1<<4);
  smbus_mem.dev_specific.ports[22] = _SFR_IO_ADDR(PORTC) + 32;
  pin_stash[11] = ~(1<<5);

  /*
  SET(PORTD, PD5);
  for (i = 0; i < 8; ++i) {
    smbus_mem.dev_specific.stopbytes[i*2+0] = 0xfb;//eeprom_read_byte((i<<1)+1);
    smbus_mem.dev_specific.stopbytes[i*2+1] = 0x02;//eeprom_read_byte((i<<1)+0);
    smbus_mem.dev_specific.ports[i*2] = _SFR_IO_ADDR(PORTA) + 32;
    smbus_mem.dev_specific.pins[i*2] = ~(1<<i);
    // servo_channels[i].servo.stopbytes[1] =  eeprom_read_byte(i<<1);
    // servo_channels[i].servo.stopbytes[0] =  eeprom_read_byte((i<<1) + 1);
    //// servo_channels[i].servo.stop = ((ASM1700HI<<8)|(ASM1700LO)) - MIDPULSE;
    // servo_channels[i].port = _SFR_IO_ADDR(PORTA) + 32;
    // servo_channels[i].pin = ~(1<<i);
  }
  for (i = 8; i < SERVO_AMOUNT; ++i) {
    smbus_mem.dev_specific.stopbytes[i*2+0] = 0x58;//eeprom_read_byte((i<<1)+1);
    smbus_mem.dev_specific.stopbytes[i*2+1] = 0x02;//eeprom_read_byte((i<<1)+0);
    smbus_mem.dev_specific.ports[i*2] = _SFR_IO_ADDR(PORTC) + 32;
    smbus_mem.dev_specific.pins[i*2] = ~(1<<(i - 6));
    // servo_channels[i].servo.stopbytes[1] =  eeprom_read_byte(i<<1);
    // servo_channels[i].servo.stopbytes[0] =  eeprom_read_byte((i<<1) + 1);
    //// servo_channels[i].servo.stop = ((ASM1700HI<<8)|(ASM1700LO)) - MIDPULSE;
    // servo_channels[i].port = _SFR_IO_ADDR(PORTC) + 32;
    // servo_channels[i].pin = ~(1<<(i - 6));
  }
  DDRA = 0xFF;
  PORTA = 0x00;
  DDRC |= (1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);
  PORTC &= ~((1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5));

  // servo_channels[0].servo.stop = ((ASM1700HI<<8)|(ASM1700LO)) - 328;
  CLR(PORTD, PD5);
  */
  // TODO add offsets per pin based on their pos in cycle
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
}


// -----------------------------------------------------------------------------
//                                                                           ADC
// -----------------------------------------------------------------------------
inline void init_ADC() {
  ADMUX = (1<<REFS1) | (1<<REFS0);
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // prescale 32
  ACSR = (1<<ACD);
  
}


// -----------------------------------------------------------------------------
//                                                                     heartbeat
// -----------------------------------------------------------------------------
void heartbeat() {
  /*
   * 0  15 30  45 100
   * 1  0  1   0  loop
   */
  if (servo_cycle_counter < 10) {
    HEARTBEAT_LED_ON();
  } else if (servo_cycle_counter < 19) {
    HEARTBEAT_LED_OFF();
  } else {
    servo_cycle_counter = 255;
  }
}

// -----------------------------------------------------------------------------
//                                                                       execute
// -----------------------------------------------------------------------------
// inline void execute(){
  // register uint8_t i;
  // switch(command){
  // case I2C_RESET:
    // reset = 1;
    // break;
  // case I2C_LOAD_STARTPOS:
    // cli();
    // for (i = 0; i < BUFLEN_SERVO_DATA; ++i) {
      // recv[i] = eeprom_read_byte(i);
    // }
    // sei();
    // break;
  // case I2C_SAVE_STARTPOS:
    // cli();
    // for (i = 0; i < BUFLEN_SERVO_DATA; ++i) {
      // eeprom_write_byte(i, servo_channels[i>>1].servo.stopbytes[1-i%2]);
    // }
    // sei();
    // break;
  // }
  // command=0;
// }


// =============================================================================
//                                                                          main
// =============================================================================
int main() {
  // green power led on PC6
  SET(DDRC, PC6);
  POWER_LED_ON();
  // green heartbeat led on PC7
  SET(DDRC, PC7);
  HEARTBEAT_LED_OFF();
  // red error led on PB0
  SET(DDRB, PB0);
  ERROR_LED_OFF();
  // comm sync pin
  SET(DDRB, PB1);
  COMM_SYNC_LOW();
  // smbus values
  smbus_mem.base.dev_type_major = 2;
  smbus_mem.base.dev_type_minor = 12;
  smbus_mem.base.version_major = 1;
  smbus_mem.base.version_minor = 0;
  smbus_mem.base.I2C_addr = 0x48;
  smbus_mem.dev_specific.servo_count = 12;
  // smbus_mem.dev_specific.undervoltage_level = 556; // 3.5 * (6.6/16.6/2.56*1024)
  smbus_mem.dev_specific.undervoltage_level = 447; // 3.0 * (6.6/16.6/2.56*1024)
  smbus_mem.dev_specific.last_initialized = -1;
  smbus_mem.dev_specific.state = STATE_INITIALIZING;
  init_smbus(&smbus_mem);
  // setup channels and timers
  init_channels();
  init_timers();
  init_ADC();
  group_ptr = &smbus_mem.dev_specific.stopbytes[0];
  servo_cycle_counter = 0;
  // go
  sei();

  wdt_enable(WDTO_1S);
  while(1) {
    if (!reset) {
      wdt_reset();
      switch (smbus_mem.dev_specific.state) {
        case STATE_RUNNING: {
          // keep up the heartbeat
          break;
        }
        case STATE_INITIALIZING: {
          heartbeat();
          if (smbus_mem.dev_specific.last_initialized <
              smbus_mem.dev_specific.servo_count) {
            if (servo_cycle_counter >= 20) {
              register uint8_t a = smbus_mem.dev_specific.last_initialized;
              ++smbus_mem.dev_specific.last_initialized;
              smbus_mem.dev_specific.pins[a*2] = pin_stash[a];
              servo_cycle_counter = 0;
            }
          } else {
            smbus_mem.dev_specific.state = STATE_RUNNING;
            HEARTBEAT_LED_ON();
          }
          break;
        }
        case STATE_ERROR: {
          cli();
          break;
        }
      }
      if (!CHK(ADCSRA, ADSC)) {
        smbus_mem.dev_specific.input_voltage = ADCW;
        ADCSRA |= (1<<ADSC);
        // TODO flip chan
      }
      if (smbus_mem.dev_specific.input_voltage < 
          smbus_mem.dev_specific.undervoltage_level &&
          smbus_mem.dev_specific.last_initialized >= 0) {
        HEARTBEAT_LED_OFF();
        ERROR_LED_ON();
        smbus_mem.dev_specific.state = STATE_ERROR;
      }
      if (CHK(TWCR,TWINT)) {
        i2cstuff(&smbus_mem);
      }
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
  TWCR = 0;
  COMM_SYNC_LOW();
  __asm__(
          "CLI \n\t"
          // set all ports of group high
          // first channel
          // load pin address on Y
          // "LDD r28, %a4+2 \n\t"
          "LDD r28, %a4+24 \n\t"
          "MOV r29, __zero_reg__ \n\t"
          "MOV r2, r28 \n\t" // also store pin addr in r2
          // read pin value
          "LD r8, Y \n\t"
          // load proper pin value from channel.pin
          // "LDD r3, %a4+3 \n\t"
          "LDD r3, %a4+48 \n\t"
          // create new value for port
          "LDI r24, 0xFF \n\t"
          "EOR r24, r3 \n\t" // invert r3
          "OR r24, r8 \n\t"
          // write to port
          "ST Y, r24 \n\t"

          // repeat for second and third channel
          // "LDD r28, %a4+6 \n\t"
          "LDD r28, %a4+26 \n\t"
          "MOV r4, r28 \n\t"
          "LD r8, Y \n\t"
          // "LDD r5, %a4+7 \n\t"
          "LDD r5, %a4+50 \n\t"
          "LDI r24, 0xFF \n\t"
          "EOR r24, r5 \n\t"
          "OR r24, r8 \n\t"
          "ST Y, r24 \n\t"

          // "LDD r28, %a4+10 \n\t"
          "LDD r28, %a4+28 \n\t"
          "MOV r6, r28 \n\t"
          "LD r8, Y \n\t"
          // "LDD r7, %a4+11 \n\t"
          "LDD r7, %a4+52 \n\t"
          "LDI r24, 0xFF \n\t"
          "EOR r24, r7 \n\t"
          "OR r24, r8 \n\t"
          "ST Y, r24 \n\t"

          // wait 400 us
          "LDI r25, %0 \n\t"
          "LDI r24, %1 \n\t"
         "loopje: \n\t"
          // "WDR \n\t"
          "SBIW r24, 0x01 \n\t"
          "BRNE loopje \n\t"
          // load counter value for 1700 us
          "LDI r25, %2 \n\t"
          "LDI r24, %3 \n\t"
          "LD  r16, %a4   \n\t"   // 1
          "LDD r17, %a4+1 \n\t"// 1
          "LDD r18, %a4+2 \n\t"   // 1
          "LDD r19, %a4+3 \n\t"// 1
          "LDD r20, %a4+4 \n\t"   // 1
          "LDD r21, %a4+5 \n\t"// 1

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
          "AND r9, r8 \n\t" // TODO(michiel): no need for r9 really
          "ST Y, r9 \n\t"

          "SEI \n\t"
  ::"M"(ASM400HI),"M"(ASM400LO),
    "M"(ASM1700HI),"M"(ASM1700LO),
    "z"(group_ptr):"r2","r3","r4","r5","r6","r7","r8","r9","r16","r17","r18",
    "r19","20","r21","r24","r25","r28","r29");

  // point to the next 3 channels for the next timer interrupt
  if (group_ptr == &smbus_mem.dev_specific.stopbytes[0]) {
    group_ptr = &smbus_mem.dev_specific.stopbytes[6];
    OCR0 = SHORT_INTERVAL;
  } else if (group_ptr == &smbus_mem.dev_specific.stopbytes[6]) {
    group_ptr = &smbus_mem.dev_specific.stopbytes[12];
    // OCR0 = SHORT_INTERVAL;
  } else if (group_ptr == &smbus_mem.dev_specific.stopbytes[12]) {
    group_ptr = &smbus_mem.dev_specific.stopbytes[18];
    // OCR0 = SHORT_INTERVAL;
  } else if (group_ptr == &smbus_mem.dev_specific.stopbytes[18]) {
    group_ptr = &smbus_mem.dev_specific.stopbytes[0];
    OCR0 = REMAINING_TIME;
    ++servo_cycle_counter;
    TWCR = (1<<TWEN) | (1<<TWEA) | I2CIE;
    COMM_SYNC_HIGH();
  }
}

// ISR(TWI_vect) {
  // i2cstuff(&smbus_mem);
// }