/* usage:
 * 1. setup dev_specific.h with your memmap
 * 2. create a global smbbus_mem_t variable
 * 3. setup the interrupt vector:
 *     ISR(TWI_vect) {
 *       i2cstuff(&smbus_mem);
 *     }
 * 4. implement these functions:
 *    void smbus_receive_command(uint8_t command);
 *    void init_smbus(smbus_mem_t* smbus_mem);
 * 5. initialize at least I2C_addr:
 *     smbus_mem.base.version_major = 0;
 *     smbus_mem.base.version_minor = 1;
 *     smbus_mem.base.dev_type_major = 1;
 *     smbus_mem.base.dev_type_minor = 1;
 *     smbus_mem.base.I2C_addr = 0x48;
 * 6. call init_smbus(&smbus_mem) and sei()
 * 
 * 
 */
#ifndef _SMBUS_SLAVE_H_
#define _SMBUS_SLAVE_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#include "smbus_slave/dev_specific.h"

#define SET(x,y) ((x)|=(1<<(y)))
#define CLR(x,y) ((x)&=(~(1<<(y))))
#define CHK(x,y) ((x)&(1<<(y))) 
#define TOG(x,y) ((x)^=(1<<(y)))

#define I2CIE (0<<TWIE)
#define NO_READMASK

typedef struct {
  uint8_t dev_type_major;  // 0
  uint8_t dev_type_minor;  // 1
  uint8_t version_major;   // 2
  uint8_t version_minor;   // 3
  uint8_t I2C_addr;        // 4
} smbus_basic_mem_t;

typedef struct {
  uint8_t data_count;
  uint8_t addr;
  union {
    uint8_t bytes[sizeof(smbus_basic_mem_t) + sizeof(smbus_dev_specific_mem_t)];
    struct {
      smbus_basic_mem_t base;
      smbus_dev_specific_mem_t dev_specific;
    };
  };
} smbus_mem_t;
#define SMBUS_MAX_ADDR (sizeof(smbus_basic_mem_t) + \
                        sizeof(smbus_dev_specific_mem_t) - 1)
void i2cstuff(smbus_mem_t* smbus_mem);

void smbus_block_write_done();
// called after a smbus "block write"

void smbus_receive_command(uint8_t command);
// called after a smbus "send byte"

void init_smbus(smbus_mem_t* smbus_mem);
// call this to set the proper registers

#endif  // _SMBUS_SLAVE_H_