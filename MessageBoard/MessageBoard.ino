/********************************************************************************

MessageBoard.ino

Internet-enabled scrolling LED message display sign designed around the Arduino
YUN for Deanshanger Sports & Social Club

Version 1.0

Copyright (c) 2014 - 2016 Richard Morris.  All Rights Reserved
https://plus.google.com/+RichardMorris_UK

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

********************************************************************************/

#include "arduino.h"

#include <avr/pgmspace.h>
#include <TimerOne.h>
#include <Bridge.h>
#include <Mailbox.h>
#include <SPI.h>

#include "font8x5.h"
#include "fastdigital.h"
#include "UTF8.h"

// enable for animation debugging
//#define ANIM_TEST 1

// for crash recovery/serial debugging
#define DEBUG 1


// =====================================
// ====== Arduino Pin Assignments ======
// =====================================

// enable debugging of setup() function
#define PIN_DEBUG_SETUP         A0

// enable debugging of main loop() function
#define PIN_DEBUG_LOOP          A1

// temperature sensor input pin (optional)
#define PIN_TEMPERATURE_SENSOR  A5

// shift registers (SNL4LS164) not clear pin
#define PIN_SR_CLEAR            12

// serial column data uses ICSP header
// row selector pins (digital ports via Darlington Driver Array)
// pins 0-1 should not be used
// pins 2-3 reserved for future use
int const row_select_pins[] = { 4, 5, 6, 7, 8, 9, 10, 11 };


// =====================================
// ======== General Definitions ========
// =====================================

#define TIMER_REFRESH_RATE    1700

// temperature sensor
// analog reference voltage
#define AREF_VOLTAGE          3.3

// matrix dimensions
#define MATRIX_ROWS           8
#define MATRIX_COLUMNS        120

// matrix zones - memory layout of display buffer
#define MATRIX_NUM_ZONES      ( MATRIX_COLUMNS / 8 )
#define MATRIX_MAX_ZONE_INDEX ( MATRIX_NUM_ZONES - 1 )

// total characters displayable on this display
#define MATRIX_MAX_CHRS       ( MATRIX_COLUMNS / FONT_PITCH )

// DrawText Alignments
#define DT_ALIGN_LEFT         0x00
#define DT_ALIGN_CENTER       0x01
#define DT_ALIGN_RIGHT        0x02

// DrawText Styles
#define DT_STYLE_NORMAL       0x00
#define DT_STYLE_INVERSE      0x01

// Scroll Direction
#define DIRECTION_LEFT        0x00
#define DIRECTION_RIGHT       0x01
#define DIRECTION_UP          0x02
#define DIRECTION_DOWN        0x03

// default message context parameters
#define DEF_ANIM_RTN          0         // smooth scrolling
#define DEF_ANIM_SPEED        5         // medium speed
#define DEF_PAUSE_INTERVAL    20        // 20 seconds delay

// display modes
#define MODE_NORMAL           0x00
#define MODE_STREAMING        0x01
#define MODE_DISPLAY_OFF      0x02

// total number of messages held in datastore
#define MSG_DATASTORE_CAPICITY 20

// datastore keys
#define KEY_DISPLAY_CONTROL    "0"
#define KEY_UPDATE_EEPROM      "1"
#define KEY_PAUSE_INTERVAL     "2"


// =====================================
// =====  Global variables in SRAM  ====
// =====================================

// for faster digital writes in Interrupt Service Routine (ISR)
static atmega_gpio_t gRowSelector[MATRIX_ROWS];
static atmega_gpio_t *isr_io = &gRowSelector[0];

// row currently being displayed by ISR
static int isr_row = 0;

// matrix frame buffer
// bits in this array represents one LED of the matrix display
// which is divided into 'zones' that are logically 8x8 units
// with one byte corresponding to eight columns
volatile static byte display_buffer[MATRIX_ROWS][MATRIX_NUM_ZONES];

struct {
  // animation properties
  unsigned int    routine = DEF_ANIM_RTN;
  unsigned long   animation_speed = DEF_ANIM_SPEED;
  unsigned long   anim_prev_millis = 0L;
  // animation state control
  int             state = 0;
  // general purpose vars used by animation routines
  int             tmp_1, tmp_2 = 0;
  // start of message pointer and length
  const char*     msg_ptr;
  unsigned int    msg_length = 0;
  // for transition delay between messages
  unsigned long   interval = 0L;
  unsigned long   delay_prev_millis = 0L;
} msg_context;

// current display mode
byte display_mode = MODE_NORMAL;

// for keeping track of messages in the datastore
byte msg_index = 0;

// when true, fetches next message
bool fetch_next_msg = true;

// small temporary buffer for datastore keynames/etc
// also used for temperature/date-time substitutions
#define TMP_BUFFER_SIZE 30
char tmp_buffer[TMP_BUFFER_SIZE];

// Main message buffer
#define MSG_BUFFER_SIZE 550
char msg_buffer[MSG_BUFFER_SIZE];


// =====================================
// ===  Global variables in PROGMEM  ===
// =====================================

// pac-man animation frames
const unsigned char pacman[2][8] PROGMEM = {
  {0x00,0x3C,0x7E,0xFF,0xFF,0xFF,0x7E,0x3C},
  {0x00,0x42,0xE7,0xFF,0xFF,0xFF,0x7E,0x3C}
};

// shown when display is starting up and waiting for Linux
// process to become available
const char startup_message[] PROGMEM =
  { "MessageBoard" };

// default message shown when we have nothing else to display
const char default_message[] PROGMEM =
  { "*** Welcome to Deanshanger Sports & Social Club ***" };


// =====================================
// ======  EEPROM message memory   =====
// =====================================

// memory layout:
//
// 0x0000:         EEPROM_MAGIC_ID
// 0x0001-0x03FC:  6x Messages of 170 bytes
// 0x03FD:         unused memory, 3 bytes
//
// Alternate memory configurations can be used for instance 7x146, 5x204, or 4x255 bytes
// Start addresses are pre-calculated in the lookup table below, and will require
// updating for these alternate configurations.
//
// Messages are generally stored as zero-terminated strings, except when the message is of
// maximum length. Zero-length messages implies vacant slot.

#define EEPROM_MAGIC_ID   0x95
#define EEPROM_MAGIC_ADDR 0x0000

#define EEPROM_MSG_MEMORY 6
#define EEPROM_MSG_LENGTH 170

const uint16_t eeprom_msg_start_addr[EEPROM_MSG_MEMORY] = { 0x01, 0xAB, 0x155, 0x1FF, 0x2A9, 0x353 };


// =====================================
// ====  Interrupt Service Routine  ====
// =====================================

// Main Interrupt Service Routine
// called by TimerOne when refresh counter overflows
//  and refreshes a single matrix row at any given time
void refresh_matrix_row(void)
{
  // hide previous row
  *(isr_io->port_register) &= isr_io->bit_mask_low;

  // advance to next row
  if (++isr_row == MATRIX_ROWS)
    isr_row = 0;
  isr_io = &gRowSelector[isr_row];

  // shift out bit pattern to matrix display
  for (int zone = MATRIX_MAX_ZONE_INDEX; zone >= 0; zone--) {
    // transfer using hardware SPI
    SPIwrite( display_buffer[isr_row][zone] );
  }

  // now make row visible
  *(isr_io->port_register) |= isr_io->bit_mask_high;
}


// =====================================
// == Support Functions begin here   ===
// =====================================

// returns current temperature sensor reading in Degrees C from thermometer pin
//  caller responsible for converting to Degrees F using formula (tempC * 9.0 / 5.0) + 32.0
//  suitable sensors:  TMP35/TMP36/TMP37
float read_temperature_sensor(void)
{
  int sensor_value = analogRead(PIN_TEMPERATURE_SENSOR);
  // converting that reading to voltage, which is based off the reference voltage
  float voltage = (sensor_value * AREF_VOLTAGE) / 1024.0;
  // convert the millivolts to temperature Celsius:
  return (voltage - 0.5) * 100 ;
}

// removes all space characters
void remove_spaces(char* src)
{
  char* i = src;
  char* j = src;
  while(*j != 0) {
    *i = *j++;
    if (*i != ' ')
      i++;
  }
  *i = 0; // zero terminator
}

// initializes IO pin for faster digital outputs
// by caching ports/registers
void set_fast_pin_output(const uint8_t pin, atmega_gpio_t &io)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  io.port_register = portOutputRegister(digitalPinToPort(pin));
  io.bit_mask_high = digitalPinToBitMask(pin);
  io.bit_mask_low  = io.bit_mask_high ^ 0xFF;
}

// specialised SPI method - write only
inline void SPIwrite(const byte data)
{
  SPDR = data;
  while (!(SPSR & _BV(SPIF)));
}

// Converts column/row to actual buffer bit and turns it on/off
void matrix_draw_pixel(const int column, const int row, const int value)
{
  if (column < 0 || column > MATRIX_COLUMNS-1 || row < 0 || row > MATRIX_ROWS-1)
     return;
  int const zone = column / 8;
  int const bit_index = column % 8;
  if (value)
    display_buffer[row][zone] |= (1 << bit_index);
  else
    display_buffer[row][zone] &= ~(1 << bit_index);
}

// draw character normal style, or inverted
void matrix_draw_chr(const int column, char c, const bool inv)
{
  if (c < FONT_FIRST_CHR || c > FONT_LAST_CHR)
    c = 0x7F;  // unknown glyph

  c -= 0x1D; // force into range of our font table (which starts at 0x1D)

  for (int col = 0; col < FONT_PITCH; col++) {
    for (int row = 0; row < MATRIX_ROWS; row++) {
      byte d = bitRead( pgm_read_byte( &Font8x5[(byte)c][col] ), row );
      if (inv) d = !d;
      matrix_draw_pixel( col + column, row, d );
    }
  }
}

// draw formatted text in normal/inverse styles with various alignment options
void matrix_draw_text(const char text[], const int count, const int format, const int style)
{
  int column, start_index;

  switch (format)
  {
    case DT_ALIGN_CENTER:
       if (count <= MATRIX_MAX_CHRS)
       {
        column = (MATRIX_COLUMNS / 2) - ((count * FONT_PITCH) / 2);
        start_index = 0;
       }
       else
       {
        column = 0;
        start_index = (count - MATRIX_MAX_CHRS) / 2;
       }
       break;

    case DT_ALIGN_RIGHT:
      if (count <= MATRIX_MAX_CHRS)
      {
       column = MATRIX_COLUMNS - (count * FONT_PITCH);
       start_index = 0;
      }
      else
      {
        column = 0;
        start_index = count - MATRIX_MAX_CHRS;
      }
      break;
 
    case DT_ALIGN_LEFT:
    default:
       column = 0;
       start_index = 0;
       break;
  }

  //Serial.print("length=");
  //Serial.println(length);
  //Serial.print("column=");
  //Serial.println(column);
  //Serial.print("index=");
  //Serial.println(start_index);

  for (int i = start_index; i < count && column < MATRIX_COLUMNS; i++) {
    matrix_draw_chr( column, text[i], style );
    column += FONT_PITCH;
  }
}

// clear/fill display buffer
void matrix_fill_buffer(const byte value)
{
  for (int row = 0; row < MATRIX_ROWS; row++) {
    for (int zone = 0; zone <= MATRIX_MAX_ZONE_INDEX; zone++) {
      display_buffer[row][zone] = value;
    }
  }
}

// rotates entire buffer left or right
void matrix_rotate_buffer(const int roll_dir)
{
  byte t;
  switch (roll_dir)
  {
    case DIRECTION_LEFT:
      for (int row = 0; row < MATRIX_ROWS; row++) {
        t = display_buffer[row][0];
        for (int zone = 0; zone < MATRIX_NUM_ZONES; zone++) {
          display_buffer[row][zone] = display_buffer[row][zone] >> 1;
          // Roll over lowest bit from the next zone as highest bit of this zone.
          if (zone < MATRIX_MAX_ZONE_INDEX) 
             bitWrite(display_buffer[row][zone], 7, bitRead(display_buffer[row][zone+1], 0));
          else
             bitWrite(display_buffer[row][zone], 7, bitRead(t, 0));
        }
      }
      break;

    case DIRECTION_RIGHT:
      for (int row = 0; row < MATRIX_ROWS; row++) {
        t = display_buffer[row][MATRIX_MAX_ZONE_INDEX];
        for (int zone = MATRIX_MAX_ZONE_INDEX; zone >=0; zone--) {
          display_buffer[row][zone] = display_buffer[row][zone] << 1;
          // Roll over highest bit from the next zone as lowest bit of this zone.
          if (zone > 0) 
             bitWrite(display_buffer[row][zone], 0, bitRead(display_buffer[row][zone-1], 7));
          else
             bitWrite(display_buffer[row][zone], 0, bitRead(t, 7));
        }
      }
      break;
  }
}

// shifts display buffer left/right/up/down
void matrix_shift_buffer(const int shift_dir)
{
  switch (shift_dir)
  {
    case DIRECTION_LEFT:
      for (int row = 0; row < MATRIX_ROWS; row++) {
        for (int zone = 0; zone < MATRIX_NUM_ZONES; zone++) {
          display_buffer[row][zone] = display_buffer[row][zone] >> 1;
          // Roll over lowest bit from the next zone as highest bit of this zone.
          if (zone < MATRIX_MAX_ZONE_INDEX) 
             bitWrite(display_buffer[row][zone], 7, bitRead(display_buffer[row][zone+1], 0));
        }
      }
      break;

   case DIRECTION_RIGHT:
      for (int row = 0; row < MATRIX_ROWS; row++) {
        for (int zone = MATRIX_MAX_ZONE_INDEX; zone >=0; zone--) {
          display_buffer[row][zone] = display_buffer[row][zone] << 1;
          // Roll over highest bit from the next zone as lowest bit of this zone.
          if (zone > 0) 
             bitWrite(display_buffer[row][zone], 0, bitRead(display_buffer[row][zone-1], 7));
        }
      }
      break;

   case DIRECTION_UP:
      for (int row = 0; row < MATRIX_ROWS; row++) {
        for (int zone = 0; zone < MATRIX_NUM_ZONES; zone++) {
          display_buffer[row][zone] = (row < 7 ? display_buffer[row+1][zone] : 0x00);
        }
      }
      break;

   case DIRECTION_DOWN:
      for (int row = MATRIX_ROWS-1; row >= 0; row--) {
        for (int zone = 0; zone < MATRIX_NUM_ZONES; zone++) {
          display_buffer[row][zone] = (row > 0 ? display_buffer[row-1][zone] : 0x00);
        }
      }
      break;
  }
}

// shifts display buffer left/right, adding in new character column data
void matrix_shiftin_chr(char c, const int bit_index, const int shift_dir)
{
  // sanity check
  if (bit_index < 0 || bit_index > 7)
    return;

  if (c < FONT_FIRST_CHR || c > FONT_LAST_CHR)
    c = 0x7F;  // unknown glyph

  c -= 0x1D; // force into range of our font table (which starts at 0x1D)

  // first, shift entire buffer in direction of travel
  matrix_shift_buffer(shift_dir);

  // then, add bit pattern for new character
  int const column = (shift_dir == DIRECTION_LEFT ? 119 : 0);
  for (int row = 0; row < MATRIX_ROWS; row++) {
    matrix_draw_pixel( column, row, bitRead( pgm_read_byte( &Font8x5[(byte)c][bit_index] ), row) );
  }
}

// start animation - must be called before do_annimation()
//  routine         = specifies the animation routine
//  animation_speed = 0 [fastest] - 10 [slowest]
//  pause_interval  = time delay in seconds
//  msg_start_prt   = pointer to message buffer
void start_animation(const int routine, const unsigned int animation_speed, const unsigned int pause_interval, const char *msg_start_ptr)
{
#if DEBUG
  Serial.print("routine=");
  Serial.print(routine);
  Serial.print(" speed=");
  Serial.print(animation_speed);
  Serial.print(" message='");
  Serial.print(msg_start_ptr);
  Serial.println("'");
#endif

  // assign animation routine
  msg_context.routine = routine;

  // timings
  msg_context.animation_speed = animation_speed;
  msg_context.interval = 1000 * pause_interval;

  msg_context.msg_ptr = msg_start_ptr;
  msg_context.msg_length = strlen(msg_start_ptr);
  if (msg_context.msg_length == 0)
    return;   // nothing to do

  // state for routine initialization
  msg_context.state = 0;
  msg_context.anim_prev_millis = 0L;

  // reset pause delay
  msg_context.delay_prev_millis = millis();

  // don't fetch messages until routine completes
  fetch_next_msg = false;
}

void do_animation(void)
{
  // get current milliseconds for animation timings
  unsigned long l_millis = millis();

   switch (msg_context.routine)
   {
     case 0:  // smooth scrolling (with clear screen)
       if (msg_context.anim_prev_millis == 0L)
         matrix_fill_buffer(0x00);
     case 1:  // smooth scrolling (without clear screen)
       if (msg_context.anim_prev_millis == 0L)
       {
        msg_context.tmp_1 = 0;  // character bit index
        msg_context.anim_prev_millis = l_millis;
       }
      else if (l_millis - msg_context.anim_prev_millis > ( msg_context.animation_speed * 10) + 15)
      {
       if (msg_context.state <= msg_context.msg_length-1)
       {
         // shift in new character bits
         matrix_shiftin_chr( msg_context.msg_ptr[msg_context.state], msg_context.tmp_1, DIRECTION_LEFT );
         if (++msg_context.tmp_1 == FONT_PITCH)
         {
          // next character in sequence
          msg_context.state++;
          msg_context.tmp_1 = 0;
         }
       }
       else
       {
         // shift out trailing characters
         matrix_shift_buffer( DIRECTION_LEFT );
         if (msg_context.state++ >= msg_context.msg_length + MATRIX_COLUMNS)
           fetch_next_msg = true;
       }
       msg_context.anim_prev_millis = l_millis;
      }
      return;
      break;

     case 2:  // static text (left alignment    )
     case 3:  // static text ( center alignment )
     case 4:  // static text (   right alignment)
       if (msg_context.anim_prev_millis == 0L)
       {
         if (msg_context.msg_length < MATRIX_MAX_CHRS)
           matrix_fill_buffer(0x00);
         matrix_draw_text( msg_context.msg_ptr, msg_context.msg_length, msg_context.routine - 2, DT_STYLE_NORMAL );
         // change to prevent re-initialization
         msg_context.anim_prev_millis++;
       }
       break;

     case 5:  // blinking text (left alignment    )
     case 6:  // blinking text ( center alignment )
     case 7:  // blinking text (   right alignment)
      if (msg_context.anim_prev_millis == 0L)
      {
        if (msg_context.state == 0)
          matrix_fill_buffer(0x00);
        else
          matrix_draw_text( msg_context.msg_ptr, msg_context.msg_length, msg_context.routine - 5, DT_STYLE_NORMAL );
          
         msg_context.anim_prev_millis = l_millis;
      }
      else if (l_millis - msg_context.anim_prev_millis > 200 + ( msg_context.animation_speed * 100))
      {
        msg_context.anim_prev_millis = 0L;
        msg_context.state = !msg_context.state;
      }
      break;

     case 8:  // inverting text (left alignment    )
     case 9:  // inverting text ( center alignment )
     case 10: // inverting text (   right alignment)
      if (msg_context.anim_prev_millis == 0L)
      {
       if (msg_context.msg_length < MATRIX_MAX_CHRS)
         matrix_fill_buffer( msg_context.state == 0 ? 0x00 : 0xFF );

       matrix_draw_text( msg_context.msg_ptr, msg_context.msg_length, msg_context.routine - 8, msg_context.state == 0 ? DT_STYLE_NORMAL : DT_STYLE_INVERSE);
       
       msg_context.anim_prev_millis = l_millis;
      }
      else if (l_millis - msg_context.anim_prev_millis > 400 + ( msg_context.animation_speed * 100))
      {
        msg_context.anim_prev_millis = 0L;
        msg_context.state = !msg_context.state;
      }
      break;

     case 11: // scroll message up
     case 12: // scroll message down
       if (msg_context.anim_prev_millis == 0L)
       {
          msg_context.tmp_1 = 0;  // number rows scrolled
          msg_context.anim_prev_millis = l_millis;
       }
       else if (msg_context.state == 0 && l_millis - msg_context.anim_prev_millis > 400 + ( msg_context.animation_speed * 100))
       {
        if (msg_context.tmp_1 < 8)
        {
         matrix_shift_buffer( msg_context.routine == 11 ? DIRECTION_UP : DIRECTION_DOWN );
         msg_context.tmp_1++;
        }
        else
        {
          matrix_draw_text( msg_context.msg_ptr, msg_context.msg_length, DT_ALIGN_CENTER, DT_STYLE_NORMAL);
          msg_context.state++;
        }
        msg_context.anim_prev_millis = l_millis;
        return;
       }
       break;

     case 13: // coarse (block) scrolling
        if (msg_context.anim_prev_millis == 0L)
        {
          matrix_fill_buffer(0x00);
          msg_context.anim_prev_millis = l_millis;
        }
        else if (l_millis - msg_context.anim_prev_millis > 90 + ( msg_context.animation_speed * 100))
        {
          if (msg_context.state <= msg_context.msg_length-1 && msg_context.state < MATRIX_MAX_CHRS)
          {
            for (int i = 0; i < FONT_PITCH; i++)
              matrix_shiftin_chr( msg_context.msg_ptr[msg_context.state], i, DIRECTION_LEFT );
            msg_context.state++;
          }
          msg_context.anim_prev_millis = l_millis;
        }
       break;

     case 14: // fast zap
        if (msg_context.anim_prev_millis == 0L)
        {
          matrix_fill_buffer(0x00);
          msg_context.tmp_1 = 0;  // character index
          msg_context.tmp_2 = 0;  // character bit index
          msg_context.anim_prev_millis = l_millis;
        }
        else if (msg_context.state == 0 && l_millis - msg_context.anim_prev_millis > 1)
        {
          // shoot in message
         if (msg_context.tmp_1 <= msg_context.msg_length-1 && msg_context.tmp_1 <= MATRIX_MAX_CHRS-1)
         {
           // shift in new character
           matrix_shiftin_chr( msg_context.msg_ptr[msg_context.tmp_1], msg_context.tmp_2, DIRECTION_LEFT );
           if (++msg_context.tmp_2 == FONT_PITCH)
           {
            // next character
            msg_context.tmp_1++;
            msg_context.tmp_2 = 0;
           }
         }
         else
         {
          if (msg_context.tmp_1 < MATRIX_MAX_CHRS)
          {
            matrix_shift_buffer( DIRECTION_LEFT );
            msg_context.tmp_2++;
            if (msg_context.tmp_2 == FONT_PITCH)
            {
              msg_context.tmp_1++;
              msg_context.tmp_2 = 0;
            }
          }
          else
            msg_context.state++;  //advance to next state
         }

          msg_context.anim_prev_millis = l_millis;
        }
        else if (msg_context.state == 1 && l_millis - msg_context.delay_prev_millis > msg_context.interval)
        {
          // pause to show message
          msg_context.state++;  //advance to next state
          msg_context.tmp_1 = 0;

          msg_context.anim_prev_millis = l_millis;
        }
        else if (msg_context.state == 2 && l_millis - msg_context.anim_prev_millis > 1)
        {
          // now shoot out message
         matrix_shift_buffer( DIRECTION_LEFT );
         if (++msg_context.tmp_1 > MATRIX_COLUMNS)
           msg_context.state++;  //advance to next state
       
          msg_context.anim_prev_millis = l_millis;
        }
        else if (msg_context.state == 3)
        {
          fetch_next_msg = true;  // we're done
        }
     return;
     break;

     case 15: // dissolve screen
        if (msg_context.state < 6000)
        {
          matrix_draw_pixel( random(MATRIX_COLUMNS), random(MATRIX_ROWS), 0 );
          msg_context.state++;
          return;
        }
        else if (msg_context.state == 6000)
        {
          matrix_fill_buffer(0x00);
          matrix_draw_text(msg_context.msg_ptr, msg_context.msg_length, DT_ALIGN_LEFT, DT_STYLE_NORMAL);
          msg_context.state++;
        }
       break;

     case 16: // sliding characters
       if (msg_context.anim_prev_millis == 0L)
       {
          matrix_fill_buffer(0x00);
          msg_context.tmp_1 = (msg_context.msg_length >= MATRIX_MAX_CHRS ? MATRIX_MAX_CHRS : msg_context.msg_length)-1; // string length (constant)
          msg_context.tmp_2 = MATRIX_COLUMNS - FONT_PITCH;  // column currently being rendered
          msg_context.anim_prev_millis = l_millis;
          return;
       }
       else if (msg_context.state <= msg_context.tmp_1 && l_millis - msg_context.anim_prev_millis > 20 + ( msg_context.animation_speed * 10))
       {
          // slide in character
          matrix_draw_chr( msg_context.tmp_2, msg_context.msg_ptr[ msg_context.state ], false );

          // erase previous character
          for (int col = 0; col < FONT_PITCH; col++) {
            for (int row = 0; row < MATRIX_ROWS; row++) {
              matrix_draw_pixel( col + msg_context.tmp_2 + FONT_PITCH, row, 0 );
            }
          }

          if (msg_context.tmp_2 > (msg_context.state * FONT_PITCH))
          {
            msg_context.tmp_2 = msg_context.tmp_2 - FONT_PITCH;
          }
          else
          {
            msg_context.tmp_2 = MATRIX_COLUMNS - FONT_PITCH;  // reset column
            msg_context.state++;  // next character in sequence
          }

          msg_context.anim_prev_millis = l_millis;
          return;
       }
     break;
 
     case 17: // rolling text (left rotation)
     case 18: // rolling text (right rotation)
       if (msg_context.anim_prev_millis == 0L)
       {
          if (msg_context.msg_length < MATRIX_MAX_CHRS)
            matrix_fill_buffer(0x00);

          matrix_draw_text( msg_context.msg_ptr, msg_context.msg_length, DT_ALIGN_CENTER, DT_STYLE_NORMAL );
          msg_context.tmp_1 = 0;
          msg_context.anim_prev_millis = l_millis;
       }
       else if (msg_context.state == 0 && l_millis - msg_context.anim_prev_millis > 2000)
        {
          // short delay, then proceed to rotation
          msg_context.state++;
          msg_context.anim_prev_millis = l_millis;
        }
        else if (msg_context.state == 1 && l_millis - msg_context.anim_prev_millis > 90)
        {
         matrix_rotate_buffer( msg_context.routine == 17 ? DIRECTION_LEFT : DIRECTION_RIGHT );
         if (++msg_context.tmp_1 == MATRIX_COLUMNS)
            msg_context.state++;
         msg_context.anim_prev_millis = l_millis;
        }
        if (msg_context.state < 2)
          return;  //BUG Fix
       break;

     case 19: // pac-man
       if (msg_context.anim_prev_millis == 0L)
       {
          msg_context.tmp_1 = MATRIX_COLUMNS - 8;  // starting column
          msg_context.tmp_2 = 1;    // initial frame (open mouth)
          msg_context.anim_prev_millis = l_millis;
       }
       else if (msg_context.state < 2 && l_millis - msg_context.anim_prev_millis > 100)
       {
          // draw next frame
          for (int col = 0; col < 8; col++) {
            for (int row = 0; row < MATRIX_ROWS; row++) {
              matrix_draw_pixel( col + msg_context.tmp_1, row, bitRead( pgm_read_byte( &pacman[msg_context.tmp_2][col] ), row) );
            }
          }

          // erase previous frame
          for (int col = 0; col < 8; col++) {
            for (int row = 0; row < MATRIX_ROWS; row++) {
              matrix_draw_pixel( col + msg_context.tmp_1 + 8, row, 0 );
            }
          }

          // choose next frame to draw
          if (msg_context.tmp_2 == 0)
            msg_context.tmp_2 = 1;
          else
          {
            msg_context.tmp_2 = 0;
            msg_context.state++;
          }

          msg_context.anim_prev_millis = l_millis;
       }
       else if (msg_context.state == 2 && msg_context.tmp_1 > -8)
       {
          // move to next column
          msg_context.tmp_1 = msg_context.tmp_1 - 8;

          msg_context.state = 0;
          msg_context.anim_prev_millis = l_millis;
       }
       else if (msg_context.tmp_1 <= -8)
       {
          // now smooth scroll new message
          matrix_fill_buffer( 0x00 );
          msg_context.routine = 0;
       }
       return;
       break;

     case 20: // random
       if (msg_context.msg_length <= MATRIX_MAX_CHRS) {
        msg_context.routine = random(0, 20);
        Serial.print("random routine = ");
        Serial.println(msg_context.routine);
       }
       else
        msg_context.routine = 0;
      break;

    default:  // unhandled animation
      fetch_next_msg = true;
      return;
   }

  // handle pause interval between messages
  if (millis() - msg_context.delay_prev_millis > msg_context.interval) {
    fetch_next_msg = true;
  }
}

#ifdef NEVER
void show_test_pattern()
{
  // fill buffer with entire character set
  char *p = msg_buffer;
  int c = FONT_FIRST_CHR + 1;
  while( c <= FONT_LAST_CHR ) {
    *(p++) = (char) c++;
  }
  *p = 0; // zero terminator

  start_animation( 0, 30, 0, msg_buffer );
  while ( !fetch_next_msg ) {
    do_animation();
  }
}
#endif

#ifdef NEVER
// show linux network interface configuration
void get_wifi_network_info()
{
  Process p;  // initialize a new process on Linux processor
  p.runShellCommand("/usr/bin/pretty-wifi-info.lua");

  char* s = msg_buffer;
  while (p.available() > 0) {
    const char c = p.read();
    if ( s < msg_buffer+sizeof(msg_buffer)-1)
    {
      if (c != 0x0A && c != 0x0D)
        *(s++) = c;
      else
        *(s++) = ' ';
    }
  }
  *s = 0; // zero terminator
}
#endif

void get_temperature(char scale, float *tempC)
{
  if (*tempC == -999)
    *tempC = read_temperature_sensor();

  float temp = *tempC;
  if (scale == 'F' || scale == 'f')
    temp = (temp * 9.0 / 5.0) + 32;
  //Serial.print("temp reading = ");
  //Serial.println(temp);

  // BUG! following doesn't work on Arduino
  //sprintf(tmp_buffer, "%4.2f", t);
  dtostrf( temp, 6, 2, tmp_buffer );

  // add default decoration for uppercase specifiers
  if (scale == 'F' || scale == 'C')
  {
    tmp_buffer[6] = 0x1E;
    tmp_buffer[7] = scale;
    tmp_buffer[8] = 0;
  }

  remove_spaces(tmp_buffer);
}

void get_date_time(String format)
{
  Process p;  // initialize a new process on Linux processor
  p.begin("/bin/date");
  p.addParameter(format);
  p.run();

  char* s = tmp_buffer;
  while (p.available() > 0) {
    const char c = p.read();
    // skip line feeds/carrage returns & quotation characters
    if ( c != 0x0A && c != 0x0D && c != '"' && s < tmp_buffer+sizeof(tmp_buffer)-1 )
      *(s++) = c;
  }
  *s = 0; // zero terminator

  // ensure any remaining data is cleared
  p.flush();

  //Serial.println( tmp_buffer );
  //Serial.print( "length = ");
  //Serial.println( strlen(tmp_buffer)) ;
}

int expand_macro(char *p, float *tempC)
{
  switch (*(p + 1))
  {
    case '%': memmove( p, p+1, strlen(p+1)+1 );
              return 1;

    case 'C':
    case 'c':
    case 'F':
    case 'f': get_temperature( *(p+1), tempC );
              break;

    case '0': get_date_time( "+%d/%m/%Y" );
              break;
    case '1': get_date_time( "+%d/%m/%y" );
              break;
    case '2': get_date_time( "+%d.%m.%y" );
              break;
    case '3': get_date_time( "+%Y-%d-%m" );
              break;
    case '4': get_date_time( "+\"%d %B %Y\"" );
              break;
    case '5': get_date_time( "+%H:%M" );
              break;
    case '6': get_date_time( "+%H:%M:%S" );
              break;
    case '7': get_date_time( "+\"%I:%M:%S %p\"" );
              break;
    case '8': get_date_time( "+\"%I:%M %p\"" );
              break;
    case '9': get_date_time( "+%c" );
              break;

    default:  return 1;
  }

  const int tmp_buffer_length = strlen(tmp_buffer);

  const int size = (tmp_buffer_length-2) + strlen(p+2) + 1;
  const int free = sizeof(msg_buffer) - (p-(char *)msg_buffer) - 1;

  if ( size < free )
  {
    memmove(p + tmp_buffer_length, p + 2, size);
    strncpy(p, tmp_buffer, tmp_buffer_length);
  }

  return tmp_buffer_length;
}

void update_display_mode()
{
  msg_buffer[0] = 0;
  Bridge.get( KEY_DISPLAY_CONTROL, msg_buffer, sizeof(msg_buffer) );
  if (msg_buffer[0]) {
    // has display mode changed?
    const byte m  = atoi(msg_buffer);
    if (display_mode != m) {
      // handle stream opening/closing here
      if (m != MODE_DISPLAY_OFF && display_mode == MODE_STREAMING) {
        // empty remaining mailbox messages (this may block)
        while (Mailbox.messageAvailable()) {
          Mailbox.readMessage( (uint8_t *) msg_buffer, sizeof(msg_buffer) );
        }
      }
      else if (m == MODE_STREAMING) {
        msg_index = 0;  /* not used by streaming */
      }
      else if (m == MODE_DISPLAY_OFF) {
        // clear screen for display off
        matrix_fill_buffer(0x00);
      }
      display_mode = m;
    }
  }
  else
  {
    // no data returned - assume normal mode 
    display_mode = MODE_NORMAL;
  }
}

void read_eeprom_messages()
{
  for (int i = 0; i < EEPROM_MSG_MEMORY; i++) {
    int address = eeprom_msg_start_addr[i];
    char c = eeprom_read_byte( (uint8_t *) address );
    if (c) {
      int p = 0;  // copy bytes from EEPROM into message buffer
      while( c && p < EEPROM_MSG_LENGTH) {
        msg_buffer[p++] = c;
        c = eeprom_read_byte( (uint8_t *) ++address );
      }
      msg_buffer[p] = 0;  // add zero terminator
      //Serial.println( msg_buffer );

      // put message into data store
      tmp_buffer[0] = i + 'a';
      tmp_buffer[1] = 0;
      Bridge.put( tmp_buffer, msg_buffer );

      // and publish message here
      tmp_buffer[0] = i + 'A';
      Bridge.put( tmp_buffer, msg_buffer );
    }
  }
}

void write_eeprom_message(const int msg_index)
{
  // sanity check
  if (msg_index < 0 || msg_index >= EEPROM_MSG_MEMORY)
    return;

  //Serial.print("message index = ");
  //Serial.println(msg_index);

  tmp_buffer[0] = msg_index + 'a';
  tmp_buffer[1] = 0;
  msg_buffer[0] = 0;
  Bridge.get( tmp_buffer, msg_buffer, sizeof(msg_buffer) );
  if (msg_buffer[0] != 0)
  {
    //Serial.print("msg_buffer = ");
    //Serial.println(msg_buffer);
    int slen = strlen(msg_buffer) + 1; // include zero terminator
    if (slen >= EEPROM_MSG_LENGTH)
      slen = EEPROM_MSG_LENGTH;        // truncate when maximum length is exceeded
    eeprom_update_block( (void *) msg_buffer, (void *) eeprom_msg_start_addr[msg_index], slen );
    //Serial.print("wrote = ");
    //Serial.println(msg_buffer);
  }
  else
  {
    eeprom_write_byte( (uint8_t *) eeprom_msg_start_addr[msg_index], 0x00 );
    //Serial.println("cleared message");
  }
}

void update_eeprom_messages()
{
  // read request and execute update action
  msg_buffer[0] = 0;
  Bridge.get( KEY_UPDATE_EEPROM, msg_buffer, sizeof(msg_buffer) );
  switch (msg_buffer[0])
  {
    case 'A':
      // update All messages
      for (int i = 0; i < EEPROM_MSG_MEMORY; i++) {
        write_eeprom_message( i );
      }
      //Serial.println("updated all messages");
      break;

    case 'a':
    case 'b':
    case 'c':
    case 'd':
    case 'e':
    case 'f':
      write_eeprom_message( msg_buffer[0] - 'a' );
      //Serial.println("updated eeprom");
      break;

    case 'X':
      // erase all messages
      for (int i = 0; i < EEPROM_MSG_MEMORY; i++) {
        eeprom_write_byte( (uint8_t *) eeprom_msg_start_addr[i], 0x00 );
      }
      //Serial.println("zeroed eeprom");
      break;

    default:
      return;
  }

  // acknowledge update request
  tmp_buffer[0] = 0;
  Bridge.put( KEY_UPDATE_EEPROM, tmp_buffer );
}


// =====================================
// = Main Program (setup/loop methods) =
// =====================================

void setup()
{
#ifdef DEBUG
  // jumper configuration
  pinMode(PIN_DEBUG_SETUP, INPUT);
  pinMode(PIN_DEBUG_LOOP, INPUT);

  // open serial port, sets data rate to 9600pbs
  Serial.begin(9600);

  // for serial debugging - pull A0 HIGH to enable
  if (digitalRead(PIN_DEBUG_SETUP) == HIGH) {
    while (!Serial)
      ; // wait here for serial port to open
  }
#endif

  // configure temperature sensor
  pinMode(PIN_TEMPERATURE_SENSOR, INPUT);
  // configure reference (external) voltage for
  // temperature readings
  analogReference(EXTERNAL);
  // dummy read having changed reference
  analogRead(PIN_TEMPERATURE_SENSOR);

  SPI.begin();  // wakeup SPI bus
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  // clear matrix shift registers, and keep HIGH
  pinMode(PIN_SR_CLEAR, OUTPUT);
  digitalWrite(PIN_SR_CLEAR, LOW);
  digitalWrite(PIN_SR_CLEAR, HIGH);

  // setup fast digital row selection outputs
  for (int row = 0; row < MATRIX_ROWS; row++ )
  {
    set_fast_pin_output(row_select_pins[row], gRowSelector[row]);
  }

  // clear display buffer
  matrix_fill_buffer(0x00);

  // show startup message (static text limited to screen width)
  strcpy_P(msg_buffer, startup_message);
  matrix_draw_text( msg_buffer, strlen(msg_buffer), DT_ALIGN_LEFT, DT_STYLE_NORMAL);

  // setup refresh timer
  Timer1.initialize(TIMER_REFRESH_RATE);
  Timer1.attachInterrupt(refresh_matrix_row);

  // Initialize Bridge, Mailbox and initial DataStore values
  Bridge.begin();
  Mailbox.begin();

#ifndef ANIM_TEST
  // indicates bridge intialization is now complete
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  const byte id = eeprom_read_byte( EEPROM_MAGIC_ADDR );
  if (id == EEPROM_MAGIC_ID)
  {
    read_eeprom_messages();
  }
  else
  {
    // format EEPROM with magic identifier and mark messages empty
    eeprom_write_byte( EEPROM_MAGIC_ADDR, EEPROM_MAGIC_ID );
    for (int i = 0; i < EEPROM_MSG_MEMORY; i++) {
      eeprom_write_byte( (uint8_t *) eeprom_msg_start_addr[i], 0x00 );
    }
  }

#ifdef ANIM_TEST
  matrix_fill_buffer(0x00);
  //start_animation( 3, 5, 20, "QWERTYxxxxxxxHEWLLOxxxxxxxQWERTY");
  start_animation( 3, 5, 20, "HELLO");
  //start_animation( 3, 5, 20, "TEST longer messageXYz" );
  //start_animation( 17, 5, 20, "TEST longer messageXbeyond screen length" );
#else
  // finally, clear buffer once again before entering main loop
  matrix_fill_buffer(0x00);
#endif
}

void loop()
{
#ifdef DEBUG
  // crash recovery
  // don't do main loop processing when A1 is pulled HIGH
  if (digitalRead(PIN_DEBUG_LOOP) == HIGH) {
    delay(2000);
    return;
  }
#endif

#ifdef ANIM_TEST
  if ( fetch_next_msg == true )
  {
    // LED signals animation completed successfully
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    return;
  }
  do_animation();
#else
  if ( fetch_next_msg == true )
  {
    // give Linux processor some time between messages
    delay(500);

    // handle display mode changes
    update_display_mode();

    // handle any updates required to EEPROM messages
    update_eeprom_messages();

    // now retrieve next message from datastore or mailbox queue
    //  when we're ready
    if (display_mode == MODE_NORMAL) {

      // zero terminate both temporary buffers
      msg_buffer[0] = 0;
      tmp_buffer[1] = 0;

      // read next message from datastore
      //  will try all 20 messages so could cause problems with screen blanking
      //  if this becomes an issue, we have to try another strategy
      for (byte i = 0; !msg_buffer[0] && i < MSG_DATASTORE_CAPICITY; i++) {

        // formulate datastore key from message index
        tmp_buffer[0] = (msg_index + 'A');
        // increment for next message retrieval
        if (++msg_index == MSG_DATASTORE_CAPICITY)
          msg_index = 0;

        Bridge.get(tmp_buffer, msg_buffer, sizeof(msg_buffer));
      }

      if (msg_buffer[0] != 0)
      {
        unsigned int routine = DEF_ANIM_RTN;
        unsigned int anim_speed = DEF_ANIM_SPEED;
        unsigned int pause_interval = DEF_PAUSE_INTERVAL;
        char *p_message = msg_buffer;

        // decode 'complex' message into routine/speed/text
        if (*p_message == '|')
        {
          char *p = p_message;
          routine = strtol( p+1, &p, 10 );
          anim_speed = strtol( p+1, &p, 10 );

          if (routine >= 0 && routine <= 20 && anim_speed >= 1 && anim_speed <= 8 && *p == '|')
            p_message = p+1;
        }

        // read temperature sensor only once per message
        float tempC = -999;

        // handle macro substitutions
        char *p = p_message;
        while (*p) {
          if (*p == '%')
            p += expand_macro( p, &tempC );
          else
            p++;
        }

        // datastore can be used to override pause interval
        tmp_buffer[0] = 0;
        Bridge.get(KEY_PAUSE_INTERVAL, tmp_buffer, sizeof(tmp_buffer));
        if (tmp_buffer[0]) {
            const unsigned int pause = atoi(tmp_buffer);
            if (pause > 0)
              pause_interval = pause;
        }

        // convert UTF-8 characters to extended ASCII
        utf8ascii(p_message);

        start_animation(routine, anim_speed, pause_interval, p_message);
      }
      else
      {
        // display default message stored in PROGMEM
        strcpy_P(msg_buffer, default_message);
        start_animation(DEF_ANIM_RTN, DEF_ANIM_SPEED, DEF_PAUSE_INTERVAL, msg_buffer);
      }
    }
    else if (display_mode == MODE_STREAMING && Mailbox.messageAvailable()) {
      const int len = Mailbox.readMessage( (uint8_t *) msg_buffer, sizeof(msg_buffer) );
      msg_buffer[len] = 0; //BUG FIX - zero termination was needed here!

      // convert UTF-8 characters to extended ASCII
      utf8ascii(msg_buffer);

      start_animation( DEF_ANIM_RTN, DEF_ANIM_SPEED, DEF_PAUSE_INTERVAL, msg_buffer );
    }
    else if (display_mode == MODE_DISPLAY_OFF) {
      delay(500); // wait 500ms before retrying
      return; //continue;
    }
  }
  else
  {
    do_animation();
  }
#endif
}

