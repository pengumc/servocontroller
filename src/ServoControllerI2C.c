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
  servo_channels[0].stop = 0x85ec;
}

// -----------------------------------------------------------------------------
//                                                                   init_timers
// -----------------------------------------------------------------------------
// void init_timers() {
  // using timer 0 (8 bit) with 1024 prescaler in CTC mode
  // to get about 50 Hz
  // TCCR0 = (1<<WGM01)|(1<<CS02)|(1<<CS00);
  // OCR0 = 234;  // (0.02 / (1/(12e6/1024)) = 234.375
  // SET(TIMSK, OCIE0);
  // sei();
// }

// -----------------------------------------------------------------------------
//                                                                          main
// -----------------------------------------------------------------------------
int main() {
  wdt_enable(WDTO_1S);
  init_channels();
  DDRD = (1<<PD5)|(1<<PD3);
  PORTD = (1<<PD5)|(0<<PD3); // red , green

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
   
   // seperate 2x8 bits works, 1x16 doesn't? try making it volatile
   
  __asm__("CLI \n\t"
          "LDI r23, 0x20 \n\t" // r23: PD5 high
          "EOR r22, r22 \n\t" // clear r22
          "start: \n\t"
          "LD r24, %a0 \n\t"   // 2
          "LDD r25, %a0+1 \n\t"// 2
          "EOR r22, r23 \n\t"  // 1 toggle bit 5
          "OUT 0x12, r22 \n\t" // 1 put on portd
          "loopje: \n\t"
          "WDR \n\t"            // 1 reset watchdog
          "SBIW r24, 0x01 \n\t" // 2
          "BRCS start \n\t"     // 1 (2 on true) if carry set, jump to start
          "JMP loopje \n\t"     // 3
  ::"b"(&servo_channels[0])); // loopcycles = 7, exitcycles = 11
  
}

// -----------------------------------------------------------------------------
//                                                                       TIMER 0
// -----------------------------------------------------------------------------
// ISR(TIMER0_COMP_vect){
  // __asm__("CLI \n\t"
          // "SEI \n\t"
  // :::"r24"); // start point to first channel, size of a channel, 
// }