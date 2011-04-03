//name: servocontrollerI2C.c
//Author: Michiel van der Coelen
//contact: Michiel.van.der.coelen@gmail.com
//date: wip
//tabsize: 2
//width: 80
//// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt


//MCU = atmega32
//Lfuse = EE
//Hfuse = DA (bootsector size 1024)
//F_CPU=12000000 //defined in makefile
//all comments assume this clock frequency, you'll have to recalculate most
//values when you change it
//when changing MCU, timer registers may have different names.


/*basic description
-------------------------------
Timer1 triggers an interrupt on an interval slightly larger than the maximum
pulsewidth of the servo-signals (so a bit more than 0.002 ms).
On this interrupt, TWO servo channels are set high and 
timer0 is started (much faster than timer1). Timer0 also triggers an interrupt
Each interrupt, a counter (Tic) is increased which represents a timeframe.
When the counter equals a high channel's 'stop' variable (Stop1 or Stop2),
the channel is pulled low. 
On the next interrupt from timer1, the next TWO channels are set high,
and the cycle repeats.
After the last TWO channels are set high and timer1 has another intterupt,
the RemainingTime until the next period is loaded into the timer.
*/

#include <avr/io.h> 
#include <avr/interrupt.h>
#include "i2c_header.h"
#include <avr/wdt.h>
//#include <avr/eeprom.h> //won't compile?

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x&(1<<y)) 
#define TOG(x,y) (x^=(1<<y))

#define SLOW_MINUS 22
#define SLOW_PLUS 33

#define SERVO_AMOUNT 12 
//even only, up to 16.
//You'll have to edit 'init_channels()' as well when changing this.

#define SERVO_PORT_FIRST8 PORTA
#define SERVO_PORT_SECOND8 PORTC
//you can use these two in your 'init_channels()' function
//or you can define 16 unique ports if you want.


//used by timer0 (12Mhz/250)
#define SERVO_MAX_PULSE 96 // 200 us
#define SERVO_MIN_PULSE 48 // 100 us

//used by timer1 (12Mhz/256):
#define SERVO_PERIOD 937 //1999 us (50 Hz)

#define SERVO_BASE 96  //205 us
//this is the time till the next TWO servo channels are handled


typedef struct struct_multiservo{
	uint8_t stop;
	uint8_t port;
	uint8_t pin;
} ServoChannel_type;


//----------------------------------------------------------------------------
//GlobalS
//----------------------------------------------------------------------------
void (*jump_to_boot)(void) = 0x0C00;
uint8_t Index; //the current servochannel we're handling
ServoChannel_type ServoChannel[SERVO_AMOUNT];

uint8_t Stop1; //variables to hold the time at which to lower the channel
uint8_t Stop2;
uint8_t Tic; //timeframe counter

uint16_t RemainingTime; //time left after all servo's have been handled
//i2c
uint8_t r_index =0;
uint8_t recv[BUFLEN_SERVO_DATA]; //buffer to store received bytes
uint8_t t_index=0;
uint8_t tran[BUFLEN_SERVO_DATA];
uint8_t new_cmd=0;
uint8_t reset=0;
uint8_t command = 0;

//----------------------------------------------------------------------------
//Functions
//----------------------------------------------------------------------------
inline void init_I2C(){
  //load slave address
 TWAR = (0x01<<1); //we're using address 0x01 
 //enable I2C hardware
  TWCR = (1<<TWEN)|(1<<TWEA);

}

void handleI2C(){ //slave version
  //check if we need to do any software actions
  if(CHK(TWCR,TWINT)){
		
    switch(TW_STATUS){
//--------------Slave receiver------------------------------------
    //SLA_W received and acked, prepare for data receiving
		case 0x60:  
      TWACK;
      r_index =0;
      break;
    case 0x80:  //a byte was received, store it and 
                //setup the buffer to recieve another
			//check if the received buffer is a command
			if(r_index ==0){
				if(TWDR < SERVO_MIN_PULSE) {
					new_cmd = TWDR;
				}else{
					recv[r_index]= TWDR;
					new_cmd =0;
				}
			} 
			if(r_index>0){
				if(new_cmd == 0){
					recv[r_index]=TWDR;
				}
				if(new_cmd != TWDR) new_cmd = 0;
			}
      r_index++;
      //don't ack next data if buffer is full
      if(r_index >= BUFLEN_SERVO_DATA){
        TWNACK;
				command = new_cmd;
      }else {
    TWACK;
   }
   break;
    case 0x68://adressed as slave while in master mode.
              //should never happen, better reset;
      reset=1;
    case 0xA0: //Stop or rep start, reset state machine
      TWACK;
      break;
//-------------- error recovery ----------------------------------
    case 0x88: //data received  but not acked
      //should not happen if the master is behaving as expected
      //switch to not adressed mode
      TWACK;
      break;
//---------------Slave Transmitter--------------------------------
    case 0xA8:  //SLA R received, prep for transmission
		            //and load first data
			//since we're just sending the servo position bytes
			//we might as well use ServoChannel[].stop instead of tran[]
      t_index=1;
      TWDR = ServoChannel[0].stop;
      TWACK;
      break;
    case 0xB8:  //data transmitted and acked by master, load next
      TWDR = ServoChannel[t_index].stop;
      t_index++;
      //designate last byte if we're at the end of the buffer
      if(t_index >= BUFLEN_SERVO_DATA) TWNACK;
      else TWACK;
      break;
    case 0xC8: //last byte send and acked by master
    //last bytes should not be acked, ignore till start/stop
      //reset=1;
    case 0xC0: //last byte send and nacked by master 
		//(as should be)
      TWACK;
      break;
//--------------------- bus error---------------------------------
    //illegal start or stop received, reset the I2C hardware
		case 0x00: 
      TWRESET;
      break;
    }
  }
}

void eeprom_write_byte(uint8_t address, uint8_t value){
	//wait for access
	while(CHK(EECR, EEWE)){
	;
	}
	EEAR = address;
	EEDR = value;
	SET(EECR,EEMWE);
	SET(EECR, EEWE);
}

uint8_t eeprom_read_byte(uint8_t address){
	//wait for access
	while(CHK(EECR, EEWE)){
	;
	}
	EEAR = address;
	SET(EECR,EERE);
	return EEDR;
	
}


inline void init_servo_timers(void){
	TCCR1A =0;
	TCCR1B =(1<<CS12)|(1<<WGM12);
	OCR1A = SERVO_BASE;
	TCCR0 = 0;
	OCR0 = 250;
}

void init_channels(){
//you need to set your own DDR
	register uint8_t i;
	uint8_t pw=72;
	for(i=0;i<8;i++){
		pw = eeprom_read_byte(i);
		if(pw < SERVO_MIN_PULSE || pw > SERVO_MAX_PULSE){
			pw = (SERVO_MAX_PULSE + SERVO_MIN_PULSE) /2;
		}
		ServoChannel[i].port = _SFR_IO_ADDR(SERVO_PORT_FIRST8);
		ServoChannel[i].pin = i;
		//ServoChannel[i].stop =pw;
		recv[i] = pw;
	}
	for(i=8;i<SERVO_AMOUNT;i++){
		pw = eeprom_read_byte(i);
		if(pw < SERVO_MIN_PULSE || pw > SERVO_MAX_PULSE){
			pw = (SERVO_MAX_PULSE + SERVO_MIN_PULSE) /2;
		}
		ServoChannel[i].port = _SFR_IO_ADDR(SERVO_PORT_SECOND8);
		ServoChannel[i].pin = i - 6;
		//ServoChannel[i].stop = pw;
		recv[i]=pw;
	}

}

 inline void execute(){
	register uint8_t i;
	SET(PORTD,PD3); //green	

	switch(command){
	case I2C_RESET:
		reset=1;
		break;
	case I2C_LOAD_STARTPOS:
		cli();
		init_channels();
		sei();
		break;
	case I2C_SAVE_STARTPOS:
		cli();
		for(i=0;i<SERVO_AMOUNT;i++){
			eeprom_write_byte(i, ServoChannel[i].stop);
		}
		sei();
		break;
	}
	command=0;
	CLR(PORTD,PD3);
}

//----------------------------------------------------------------------------
//MAIN
//----------------------------------------------------------------------------
//usb initialization
int main() {
	//I2c
	init_I2C();
	//initialize servo channels and timers
	init_channels();
	init_servo_timers();
	SET(TIMSK, OCIE1A);
	RemainingTime = SERVO_PERIOD - (SERVO_BASE * (SERVO_AMOUNT)/2);
	//servo port data directions
	DDRA = 0xFF;
	DDRC |= (1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);
	DDRD|= (1<<PD5)|(1<<PD3);
	//start the signal generation
	wdt_enable(WDTO_1S);
	sei();
//----------------------------------------------------------------------------
//MAIN LOOP
//----------------------------------------------------------------------------
while(1){
	if(!reset) wdt_reset();
	handleI2C();
	if(command != 0) execute();
	
	
	}//main loop end
}//main end

//timer0 interrupt routine
ISR(TIMER0_COMP_vect){
	//increase timeframe counter
	Tic++;
	//check if we need to pull a channel low
	Index--;
	if(Tic == Stop2) {
		CLR(_SFR_IO8(ServoChannel[Index].port), ServoChannel[Index].pin);
	}
	Index--;
	if(Tic == Stop1) {
		CLR(_SFR_IO8(ServoChannel[Index].port), ServoChannel[Index].pin);
	}
	Index++;
	Index++;
}

//timer1 interrupt routine
ISR(TIMER1_COMPA_vect){
	//stop and reset timer0 and the counter
	TOG(PORTD,PD5);//red
	CLR(TIMSK, OCIE0);
	TCCR0 = 0;
	TCNT0=0;
	Tic=0;
	
	//restore the base interval if this is the first set of channels
	if (Index == 0) OCR1A = SERVO_BASE;

	if(Index != SERVO_AMOUNT) {
		//update if necessary
		if (recv[Index] == SLOW_PLUS){
			if (ServoChannel[Index].stop < SERVO_MAX_PULSE){
				ServoChannel[Index].stop++;
			}
			else ServoChannel[Index].stop = SERVO_MAX_PULSE;
		}else if(recv[Index]== SLOW_MINUS){
			if (ServoChannel[Index].stop > SERVO_MIN_PULSE) {
				ServoChannel[Index].stop--;
			}else ServoChannel[Index].stop = SERVO_MIN_PULSE;
		}else {
			ServoChannel[Index].stop = recv[Index];
		}
		
		//set TWO servo channels high
		SET(_SFR_IO8(ServoChannel[Index].port), ServoChannel[Index].pin);
		Stop1 = ServoChannel[Index].stop;
		Index++;
		if (recv[Index] == SLOW_PLUS){
			if (ServoChannel[Index].stop < SERVO_MAX_PULSE){
				ServoChannel[Index].stop++;
			}else ServoChannel[Index].stop = SERVO_MAX_PULSE;
		}else if(recv[Index]== SLOW_MINUS){
			if (ServoChannel[Index].stop > SERVO_MIN_PULSE){
				ServoChannel[Index].stop--;
			}else ServoChannel[Index].stop = SERVO_MIN_PULSE;
		}else {
			ServoChannel[Index].stop = recv[Index];
		}
		SET(_SFR_IO8(ServoChannel[Index].port), ServoChannel[Index].pin);
		Stop2 = ServoChannel[Index].stop;
		Index++;
		//start timer0 again
		TCCR0 = (1<<CS00)|(1<<WGM01);
		SET(TIMSK, OCIE0);
	} else { //we've done all servo channels, wait for next period
		Index=0;
		OCR1A = RemainingTime; 
	}
}

