// Arduino compiler hack - structure declared in seperate
// header when used as parameters to functions
// see http://playground.arduino.cc/Code/Struct

// for faster digital writes
//  - stores port register and bitmasks for row selectors
struct atmega_gpio_t
{
  volatile uint8_t bit_mask_high;
  volatile uint8_t bit_mask_low;
  volatile uint8_t *port_register;
} ;

