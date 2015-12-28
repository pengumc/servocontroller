#include "smbus_slave/smbus_slave.h"

void init_smbus(smbus_mem_t* smbus_mem) {
  TWAR = (smbus_mem->base.I2C_addr << 1) | 0x01;
  TWBR = 32;  // f_scl = f_clock / (16 + 2 * TWBR * prescale)
  TWCR = (1<<TWEN) | (1<<TWEA) | I2CIE;
}

void i2cstuff(smbus_mem_t* smbus_mem) {
  switch(TW_STATUS) {
    
    case TW_SR_SLA_ACK: {  // slave receiver ack
      smbus_mem->data_count = 0;
      TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | I2CIE;
      break;
    }
    
    case TW_SR_DATA_ACK: {  // data received.
      if (smbus_mem->data_count == 0) {
        // first byte. either a single byte command, or a start address
        smbus_mem->addr = TWDR;
      } else if (smbus_mem->data_count == 1) {
        // second byte. so we're doing a block
        // we can actually ignore the data length since we'll just keep going
      } else { // third, so it's a block write
        #ifdef NO_READMASK
        if (smbus_mem->addr <= SMBUS_MAX_ADDR) {
          smbus_mem->bytes[smbus_mem->addr] = TWDR;
        }
        smbus_mem->addr++;
        #endif  // NO_READMASK
      }
      TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | I2CIE;
      ++smbus_mem->data_count;
      break;
    }
    
    case TW_SR_STOP: {  // slave receiver stop
      if (smbus_mem->data_count == 1) {
        // received a single byte + stop --> single command
        smbus_receive_command(smbus_mem->addr);
      } else if (smbus_mem->data_count > 2){
        smbus_block_write_done();
      }
      TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | I2CIE;
      break;
    }
    
    case TW_ST_DATA_ACK:
    case TW_ST_SLA_ACK: {  // slave transmitter
      if (smbus_mem->addr <= SMBUS_MAX_ADDR) {
        TWDR = smbus_mem->bytes[smbus_mem->addr];
        smbus_mem->addr++;
      } else {
        TWDR = 0;
      }
      TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | I2CIE;
      break;
    }
    
    case TW_BUS_ERROR: {
      TWCR = (1<<TWEN) | (1<<TWSTO) | (1<<TWINT) | I2CIE;
      break;
    }
    default: TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | I2CIE;
  }
}

