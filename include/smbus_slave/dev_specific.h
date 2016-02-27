#ifndef _DEV_SPECIFIC_H_
#define _DEV_SPECIFIC_H_

// put whatever you want in this struct. 
// your data will be available at address 5 
// (the first 5 are reserved)
typedef struct {
  uint8_t stopbytes[24]; // byte 0 = lsb, 1 = msb, 2 = lsb again
  uint8_t ports[24]; // only even bytes are used
  uint8_t pins[24];  // only even bytes are used
  uint8_t servo_count;
  uint16_t undervoltage_level;
  uint16_t overcurrent_level;
  uint16_t input_voltage;
  uint16_t servo_current;
  int8_t last_initialized;
  uint8_t state;
} smbus_dev_specific_mem_t;

// Why are there 24 bytes for ports and pins when there's only 12 servos?
// Because the assembly part uses the address relative to the stopbyte 
// of the current servo, and that relative jump needs to be constant

/*
map:
  5  stopbyte low,  servo 0
  6  stopbyte high, servo 0
  7  stopbyte low,  servo 1
  8  stopbyte high, servo 1
  9  stopbyte low,  servo 2
  10 stopbyte high, servo 2
  11 stopbyte low,  servo 3
  12 stopbyte high, servo 3
  13 stopbyte low,  servo 4
  14 stopbyte high, servo 4
  15 stopbyte low,  servo 5
  16 stopbyte high, servo 5
  17 stopbyte low,  servo 6
  18 stopbyte high, servo 6
  19 stopbyte low,  servo 7
  20 stopbyte high, servo 7
  21 stopbyte low,  servo 8
  22 stopbyte high, servo 8
  23 stopbyte low,  servo 9
  24 stopbyte high, servo 9
  25 stopbyte low,  servo 10
  26 stopbyte high, servo 10
  27 stopbyte low,  servo 11
  28 stopbyte high, servo 11
  29 port servo 0
  30 --
  31 port servo 1
  32 --
  33 port servo 2
  34 --
  35 port servo 3
  36 --
  37 port servo 4
  38 --
  39 port servo 5
  40 --
  41 port servo 6
  42 --
  43 port servo 7
  44 --
  45 port servo 8
  46 --
  47 port servo 9
  48 --
  49 port servo 10
  50 --
  51 port servo 11
  52 --
  53 pin servo 0
  54 --
  55 pin servo 1
  56 --
  57 pin servo 2
  58 --
  59 pin servo 3
  60 --
  61 pin servo 4
  62 --
  63 pin servo 5
  64 --
  65 pin servo 6
  66 --
  67 pin servo 7
  68 --
  69 pin servo 8
  70 --
  71 pin servo 9
  72 --
  73 pin servo 10
  74 --
  75 pin servo 11
  76 --
  77 servo_count
  78 low byte  undervoltage_level
  79 high byte undervoltage_level
  80 low byte  overcurrent_level
  81 high byte overcurrent_level
  82 low byte  input_voltage
  83 high byte input_voltage
  84 low byte  servo_current
  85 high byte servo_current
  86 last_initialized;
  87 state;
*/



#endif  // _DEV_SPECIFIC_H_