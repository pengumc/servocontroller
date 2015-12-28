#ifndef _DEV_SPECIFIC_H_
#define _DEV_SPECIFIC_H_

// put whatever you want in this struct. 
// your data will be available at address 5 
// (the first 5 are reserved)
typedef struct {
  uint8_t stopbytes[24]; // byte 0 = lsb, 1 = msb, 2 = lsb again
  uint8_t ports[24]; // only even bytes are used
  uint8_t pins[24];  // only even bytes are used
} smbus_dev_specific_mem_t;

#endif  // _DEV_SPECIFIC_H_