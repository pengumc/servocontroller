#ifndef I2C_HEADER_H
#define I2C_HEADER_H
#ifndef USBCOMMANDLINE
  #include <util/twi.h>
#endif

#define I2C_SLAVE_ADDRESS 0x01
#define SLA_W ((I2C_SLAVE_ADDRESS<<1) | TW_WRITE)
#define SLA_R ((I2C_SLAVE_ADDRESS<<1) | TW_READ)
#define BUFLEN_SERVO_DATA 12
  #define SERVO_DATA_EMPTY {68,92,54,72,92,60,68,98,64,72,88,58}
#define BUFLEN_ACC_DATA 3
  #define ACC_DATA_EMPTY {0,0,0}
#define TWACK (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA))
#define TWNACK (TWCR=(1<<TWINT)|(1<<TWEN))
#define TWSTART (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTA))
#define TWRESET (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO)|(1<<TWEA))
#define TWSTOSTA (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTA)|(1<<TWSTO))
#endif
