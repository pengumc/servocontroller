#ifndef I2C_HEADER_H
#define I2C_HEADER_H
#ifndef USBCOMMANDLINE
  #include <util/twi.h>
#endif

#define I2C_SLAVE_ADDRESS 0x01
#define SLA_W ((I2C_SLAVE_ADDRESS<<1) | TW_WRITE)
#define SLA_R ((I2C_SLAVE_ADDRESS<<1) | TW_READ)
#define BUFLEN_SERVO_DATA 24
#define BUFLEN_ACC_DATA 3
#define TWACK (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA))
#define TWNACK (TWCR=(1<<TWINT)|(1<<TWEN))
#define TWSTART (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTA))
#define TWRESET (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO)|(1<<TWEA))
#define TWSTOSTA (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTA)|(1<<TWSTO))

//I2C commands
//these are issued by sending an array filled with only the define value

#define I2C_COMMAND_FAILED 255
//internally used to idenitify faulty buffers

#define I2C_RESET 4
//will reset the I2C slave

#define I2C_LOAD_STARTPOS 5
//will make the servocontroller load the array stored in eeprom

#define I2C_SAVE_STARTPOS 6
//servocontroller saves the current servopositions in EEPROM
#endif
