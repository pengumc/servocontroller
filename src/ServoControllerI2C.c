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
// 1 0 0 0 1 0 0 1 : 0x89 (wanted)
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

#define ASM1700LO 0x22
#define ASM1700HI 0x06

typedef struct {
  uint16_t stop;
  uint8_t port;
  uint8_t pin;
} ServoChannel_t;

ServoChannel_t servo_channels[SERVO_AMOUNT];
// -----------------------------------------------------------------------------
//                                                                 init_channels
// -----------------------------------------------------------------------------
void init_channels() {
  // hardcoded ports and pins
  uint8_t i = 0;
  for (i = 0; i < 8; ++i) {
    servo_channels[i].stop = 0;
    servo_channels[i].port = _SFR_IO_ADDR(PORTA);
    servo_channels[i].pin = i;
  }
  for (i = 8; i < SERVO_AMOUNT; ++i) {
    servo_channels[i].stop = 0;
    servo_channels[i].port = _SFR_IO_ADDR(PORTC);
    servo_channels[i].pin = i - 6;
  }
  DDRA = 0xFF;
  DDRC |= (1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);

  servo_channels[0].port = _SFR_IO_ADDR(PORTD);
  servo_channels[0].pin = 5;
  servo_channels[0].stop =((ASM1700HI<<8)|(ASM1700LO)) - 1015;
  // base value - (x/T_clk) / 13
  // where x is the time you want added to 400us
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
  //PORTD = (1<<PD5)|(0<<PD3); // red , green
  init_timers();
  
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
    1 clear pin
    3 jump somewhere
   ---
    9
  
  counter != stop, r17 = r25 but r16 != r24
    3 first cp skips
    1 cp
    2 brne true, so branch to noclear2
   --- 
    6 still needs 3 nops
  
  count  != stop, r17 != r25
   1 fist cp doesn't skip
   3 jump to noclear1
  ---
   4 still needs 5 nops
   
   total loopcycles = 13 (2 for sbiw, 2 for branch)
  */
  TOG(PORTD, PD3);
  __asm__(
          "CLI \n\t"
          "SBI 0x12, 5 \n\t"
          // wait 400 us
          "LDI r24, %1 \n\t"  // 1
          "LDI r25, %0 \n\t"  // 2
          "loopje: \n\t"  
          "WDR \n\t"
          "SBIW r24, 0x01 \n\t" // 2
          "BRNE loopje \n\t"  // 1 (2 on true)
          // load counter value for 1700 us
          "LDI r24, %3 \n\t"
          "LDI r25, %2 \n\t"
          "LD r16, %a4 \n\t"   // 1
          "LDD r17, %a4+1 \n\t"// 1
          "checkloop: \n\t" 
          "CPSE r17, r25 \n\t" // 1 (3 on equal since jmp is 2 words)
          "JMP noclear1 \n\t"  // 3
          "CP r16, r24 \n\t"
          "BRNE noclear2 \n\t" //
          "CBI 0x12, 5 \n\t"   // 1
          "JMP finalpart \n\t"
          "noclear1: \n\t"
          "nop \n\t"  // 1
          "nop \n\t"  // 1
          "noclear2: \n\t"
          "nop \n\t"  // 1
          "nop \n\t"  // 1
          "nop \n\t"  // 1
          "finalpart: \n\t"
          "SBIW r24, 0x01 \n\t" // 2
          "BRNE checkloop \n\t" // 1 (2 on true)
          
          "CBI 0x12, 5 \n\t" 
          "SEI \n\t"
  ::"M"(0x03),"M"(0xc0),"M"(0x06),"M"(0x22),"e"(&servo_channels[0]));
  
}