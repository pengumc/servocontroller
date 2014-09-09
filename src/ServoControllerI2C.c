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

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x&(1<<y))
#define TOG(x,y) (x^=(1<<y))

#define SERVO_AMOUNT 12

#define ASM400HI 0x03
#define ASM400LO 0xC0
#define ASM1700LO 0xA0
#define ASM1700HI 0x03


typedef struct {
  uint16_t stop;
  uint8_t port;
  uint8_t pin;
} ServoChannel_t;

ServoChannel_t servo_channels[SERVO_AMOUNT];
volatile ServoChannel_t* group_ptr;

// -----------------------------------------------------------------------------
//                                                                 init_channels
// -----------------------------------------------------------------------------
void init_channels() {
  // hardcoded ports and pins
  // use _SFR_IO_ADDR(PORTA) + 30 or _SFR_IO_ADDR(PINA) + 32
  // and ~(1<<bitno) to indicate bitno
  uint8_t i = 0;
  for (i = 0; i < 8; ++i) {
    servo_channels[i].stop = ((ASM1700HI<<8)|(ASM1700LO)) - i*55;
    servo_channels[i].port = _SFR_IO_ADDR(PORTA) + 32;
    servo_channels[i].pin = ~(1<<i);
  }
  for (i = 8; i < SERVO_AMOUNT; ++i) {
    servo_channels[i].stop = ((ASM1700HI<<8)|(ASM1700LO)) - i*55;
    servo_channels[i].port = _SFR_IO_ADDR(PORTC) + 32;
    servo_channels[i].pin = ~(1<<(i - 6));
  }
  DDRA = 0xFF;
  PORTA = 0x00;
  DDRC |= (1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);
  // DDRC = 0xff;
  PORTC &= ~((1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5));
  // PORTC = 0x00;

  servo_channels[0].stop = ((ASM1700HI<<8)|(ASM1700LO)) - 328;
  
  // servo_channels[0].stop =((ASM1700HI<<8)|(ASM1700LO)) - 327; // 1000 us
  // base value - (x/T_clk) / 34
  // where x is the time you want added to 400us
  // servo_channels[1].stop =((ASM1700HI<<8)|(ASM1700LO)) - 600; // 1500 us
  // servo_channels[2].stop =((ASM1700HI<<8)|(ASM1700LO)) - 873; // 2000 us

  // TODO add offsets per pin based on their pos in cycle
}


// -----------------------------------------------------------------------------
//                                                                   init_timers
// -----------------------------------------------------------------------------
void init_timers() {
  // using timer 0 (8 bit) with 1024 prescaler in CTC mode
  // to get about 50 Hz
  TCCR0 = (1<<WGM01)|(1<<CS02)|(1<<CS00);
  OCR0 = 234u;  // (0.02 / (1/(12e6/1024)) = 234.375
  SET(TIMSK, OCIE0);
  sei();
}

// -----------------------------------------------------------------------------
//                                                                          main
// -----------------------------------------------------------------------------
int main() {
  init_channels();
  DDRD = 0xff;//(1<<PD5)|(1<<PD3);
  PORTD = (0<<PD5)|(0<<PD3); // red , green
  init_timers();
  group_ptr = &servo_channels[0];

  /*
  - set all ddr
  - start timer for 50 Hz

  - 50 hz ISR
  - enter assembly
  - cli
  - set all pins high
  - wait 400 us
  - set counter to 1700 us
  - # loop
  - check each pin counter, set to 0 if needed counters for each pin need to be
    offset individually. make sure setting and doing nothing takes the same
    amount of cycles

    cut into groups to reduce looptime? can be done after first implementation

  - decrement counter
  - if 0 exit assembly
  - else jump to loop
  - exit assembly
  - sei
  - exit ISR

  - main loop:
  - handle I2C
  - clear wdt

  */
  wdt_enable(WDTO_1S);

  while(1) {
    wdt_reset();
  }
}

// -----------------------------------------------------------------------------
//                                                                       TIMER 0
// -----------------------------------------------------------------------------
ISR(TIMER0_COMP_vect){
  // 400e-6 / (T_clk *5) = 960 = 0x3c0
  // 2100e-6 / (T_clk *5) = 5039.9 = 0x13b0

  /*
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
  TOG(PORTD, PD3);
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
          "LDI r26, 0xFF \n\t"
          "EOR r26, r3 \n\t" // r26 <- ~r3
          "OR r26, r8 \n\t"
          // write to port
          "ST Y, r26 \n\t"

          // repeat for second and third channel
          "LDD r28, %a4+6 \n\t"
          "MOV r4, r28 \n\t"
          "LD r8, Y \n\t"       // #  #  #  #
          "LDD r5, %a4+7 \n\t"  // 1  1  0  0
          "LDI r26, 0xFF \n\t"  // 1  1  1  1
          "EOR r26, r5 \n\t"    // 0  0  1  1
          "OR r26, r8 \n\t"     // #  #  1  1
          "ST Y, r26 \n\t"

          "LDD r28, %a4+10 \n\t"
          "MOV r6, r28 \n\t"
          "LD r8, Y \n\t"
          "LDD r7, %a4+11 \n\t"
          "LDI r26, 0xFF \n\t"
          "EOR r26, r7 \n\t"
          "OR r26, r8 \n\t"
          "ST Y, r26 \n\t"


          // "LDI r24, 0xff \n\t"
          // "OUT 0x1B, r24 \n\t"  // set all pins of port group high
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
          "CPSE r17, r25 \n\t"  // from here
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
         "finalpart1: \n\t"  // to here is 9 cycles, always.

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
          "LD r8, Y \n\t"  // #   #   #   #
          "MOV r9, r7 \n\t"// 1   1   0   1
          "AND r9, r8 \n\t"// #   #   0   #
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
          "MOV r27, r2 \n\t"
          "LD r8, Y \n\t" 
          "MOV r9, r3 \n\t" 
          "AND r9, r8 \n\t" 
          "ST Y, r9 \n\t"
          "MOV r27, r4 \n\t"
          "LD r8, Y \n\t"
          "MOV r9, r5 \n\t"
          "AND r9, r8 \n\t"
          "ST Y, r9 \n\t"
          "MOV r27, r6 \n\t"
          "LD r8, Y \n\t"
          "MOV r9, r7 \n\t"
          "AND r9, r8 \n\t"
          "ST Y, r9 \n\t"

          
          "SEI \n\t"
  ::"M"(ASM400HI),"M"(ASM400LO),
    "M"(ASM1700HI),"M"(ASM1700LO),
    "z"(group_ptr):"r2","r3","r4","r5","r6","r7","r8","r9","r16","r17","r18",
    "r19","20","r21","r22","r23","r24","r28","r29");
  if (group_ptr == &servo_channels[0]) {
    group_ptr = &servo_channels[3];
    SET(PORTD, PD5);
    OCR0 = 30;
  } else if (group_ptr == &servo_channels[3]) {
    group_ptr = &servo_channels[6];
  } else if (group_ptr == &servo_channels[6]) {
    group_ptr = &servo_channels[9];
  } else if (group_ptr == &servo_channels[9]) {
    group_ptr = &servo_channels[0];
    OCR0 = 115;
    CLR(PORTD, PD5);
  }
  /*
  set timer to 200 Hz and do groups of 8?
  stick to 12 for now, 4 groups of 3, OCR0 = 58
    can't do that. need room for I2C
    let's do 4 groups at OCR0 = 30 and then go from there
  start of Interrupt, load index value
  use index value to select start of loading points for stop values
  also load ports and pins from mem...
  setting a single port:

  2 LDD r30, %servo_channels+2 (aka portno)
  2 LDI r31, __zero_reg__
  2 LDD r22, %servo_channels+3 (aka pin (already as ~(1<<pinno) ie a zero at pin#)
  2 LDD r23 Z-2 ( PIN is 2 lower than PORT)
  1 AND r22, r23
  2 ST Z, r22
 ---
 11

 + do three times before starting loops
 + LDD can only do positive, so store PIN instead of PORT

 technically, there's already a proper counter at 12MHz called counter1
 using that and checking the interrupt flag saves some cycles... but really
 not that much (maybe 2?)
  */
}