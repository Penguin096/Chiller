#include "HD44780_LiquidCrystal_I2C.h"
// #include <inttypes.h>
// #include <Arduino.h>
// #include <Wire.h>

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

HD44780_LiquidCrystal_I2C::HD44780_LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows)
{
  _Addr = lcd_Addr;
  _cols = lcd_cols;
  _rows = lcd_rows;
  _backlightval = LCD_NOBACKLIGHT;
}

// void LiquidCrystal_I2C::oled_init(){
//   _oled = true;
// 	init_priv();
// }

void HD44780_LiquidCrystal_I2C::init()
{
  init_priv();
}

void HD44780_LiquidCrystal_I2C::init_priv()
{
  // Wire.begin();
  _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  begin(_cols, _rows);
}

void HD44780_LiquidCrystal_I2C::begin(uint8_t cols, uint8_t lines, uint8_t dotsize)
{
  if (lines > 1)
  {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != 0) && (lines == 1))
  {
    _displayfunction |= LCD_5x10DOTS;
  }

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
  HAL_Delay(50); // delay(50);

  // Now we pull both RS and R/W low to begin commands
  expanderWrite(_backlightval); // reset expanderand turn backlight off (Bit 8 =1)
  HAL_Delay(1000);

  // put the LCD into 4 bit mode
  // this is according to the hitachi HD44780 datasheet
  // figure 24, pg 46

  // we start in 8bit mode, try to set 4 bit mode
  write4bits(0x03 << 4);
  HAL_Delay(5); // delayMicroseconds(4500);   // wait min 4.1ms

  //  // second try
  write4bits(0x03 << 4);
  HAL_Delay(5); //  delayMicroseconds(4500); // wait min 4.1ms

  //  // third go!
  write4bits(0x03 << 4);
  HAL_Delay(1); //  delayMicroseconds(150);

  //  // finally, set to 4-bit interface
  write4bits(0x02 << 4);

  // set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for roman languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

  home();
}

/********** high level commands, for the user! */
void HD44780_LiquidCrystal_I2C::clear()
{
  command(LCD_CLEARDISPLAY); // clear display, set cursor position to zero
  HAL_Delay(2);              // this command takes a long time!
  if (_oled)
    setCursor(0, 0);
}

void HD44780_LiquidCrystal_I2C::home()
{
  command(LCD_RETURNHOME); // set cursor position to zero
  HAL_Delay(2);            // this command takes a long time!
}

void HD44780_LiquidCrystal_I2C::setCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row > _numlines)
  {
    row = _numlines - 1; // we count rows starting w/0
  }
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void HD44780_LiquidCrystal_I2C::noDisplay()
{
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void HD44780_LiquidCrystal_I2C::display()
{
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// // Turns the underline cursor on/off
// void LiquidCrystal_I2C::noCursor() {
// 	_displaycontrol &= ~LCD_CURSORON;
// 	command(LCD_DISPLAYCONTROL | _displaycontrol);
// }
// void LiquidCrystal_I2C::cursor() {
// 	_displaycontrol |= LCD_CURSORON;
// 	command(LCD_DISPLAYCONTROL | _displaycontrol);
// }

// // Turn on and off the blinking cursor
// void LiquidCrystal_I2C::noBlink() {
// 	_displaycontrol &= ~LCD_BLINKON;
// 	command(LCD_DISPLAYCONTROL | _displaycontrol);
// }
// void LiquidCrystal_I2C::blink() {
// 	_displaycontrol |= LCD_BLINKON;
// 	command(LCD_DISPLAYCONTROL | _displaycontrol);
// }

// // These commands scroll the display without changing the RAM
// void LiquidCrystal_I2C::scrollDisplayLeft(void) {
// 	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
// }
// void LiquidCrystal_I2C::scrollDisplayRight(void) {
// 	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
// }

// // This is for text that flows Left to Right
// void LiquidCrystal_I2C::leftToRight(void) {
// 	_displaymode |= LCD_ENTRYLEFT;
// 	command(LCD_ENTRYMODESET | _displaymode);
// }

// // This is for text that flows Right to Left
// void LiquidCrystal_I2C::rightToLeft(void) {
// 	_displaymode &= ~LCD_ENTRYLEFT;
// 	command(LCD_ENTRYMODESET | _displaymode);
// }

// // This will 'right justify' text from the cursor
// void LiquidCrystal_I2C::autoscroll(void) {
// 	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
// 	command(LCD_ENTRYMODESET | _displaymode);
// }

// // This will 'left justify' text from the cursor
// void LiquidCrystal_I2C::noAutoscroll(void) {
// 	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
// 	command(LCD_ENTRYMODESET | _displaymode);
// }

// // Allows us to fill the first 8 CGRAM locations
// // with custom characters
// void LiquidCrystal_I2C::createChar(uint8_t location, uint8_t charmap[]) {
// 	location &= 0x7; // we only have 8 locations 0-7
// 	command(LCD_SETCGRAMADDR | (location << 3));
// 	for (int i=0; i<8; i++) {
// 		write(charmap[i]);
// 	}
// }

// //createChar with PROGMEM input
// void LiquidCrystal_I2C::createChar(uint8_t location, const char *charmap) {
// 	location &= 0x7; // we only have 8 locations 0-7
// 	command(LCD_SETCGRAMADDR | (location << 3));
// 	for (int i=0; i<8; i++) {
// 	    	write(pgm_read_byte_near(charmap++));
// 	}
// }

// Turn the (optional) backlight off/on
void HD44780_LiquidCrystal_I2C::noBacklight(void)
{
  _backlightval = LCD_NOBACKLIGHT;
  expanderWrite(0);
}

void HD44780_LiquidCrystal_I2C::backlight(void)
{
  _backlightval = LCD_BACKLIGHT;
  expanderWrite(0);
}

/*********** mid level commands, for sending data/cmds */

inline void HD44780_LiquidCrystal_I2C::command(uint8_t value)
{
  send(value, 0);
}

/************ low level data pushing commands **********/

// write either command or data
void HD44780_LiquidCrystal_I2C::send(uint8_t value, uint8_t mode)
{
  uint8_t highnib = value & 0xf0;
  uint8_t lownib = (value << 4) & 0xf0;
  write4bits((highnib) | mode);
  write4bits((lownib) | mode);
}

void HD44780_LiquidCrystal_I2C::write4bits(uint8_t value)
{
  expanderWrite(value);
  pulseEnable(value);
}

void HD44780_LiquidCrystal_I2C::expanderWrite(uint8_t _data)
{
  uint8_t data = (int)(_data) | _backlightval;
  HAL_I2C_Master_Transmit(&hi2c1, (_Addr << 1), &data, 1, 10);
}

void HD44780_LiquidCrystal_I2C::pulseEnable(uint8_t _data)
{
  expanderWrite(_data | En); // En high
  HAL_Delay(1);              // enable pulse must be >450ns

  expanderWrite(_data & ~En); // En low
  HAL_Delay(1);               // commands need > 37us to settle
}

// // Alias functions

// void LiquidCrystal_I2C::cursor_on(){
// 	cursor();
// }

// void LiquidCrystal_I2C::cursor_off(){
// 	noCursor();
// }

// void LiquidCrystal_I2C::blink_on(){
// 	blink();
// }

// void LiquidCrystal_I2C::blink_off(){
// 	noBlink();
// }

// void LiquidCrystal_I2C::load_custom_character(uint8_t char_num, uint8_t *rows){
// 		createChar(char_num, rows);
// }

// void LiquidCrystal_I2C::setBacklight(uint8_t new_val){
// 	if(new_val){
// 		backlight();		// turn backlight on
// 	}else{
// 		noBacklight();		// turn backlight off
// 	}
// }

// void LiquidCrystal_I2C::printstr(const char c[]){
// 	//This function is not identical to the function used for "real" I2C displays
// 	//it's here so the user sketch doesn't have to be changed
// 	print(c);
// }

// // unsupported API functions
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-parameter"
// void LiquidCrystal_I2C::off(){}
// void LiquidCrystal_I2C::on(){}
// void LiquidCrystal_I2C::setDelay (int cmdDelay,int charDelay) {}
// uint8_t LiquidCrystal_I2C::status(){return 0;}
// uint8_t LiquidCrystal_I2C::keypad (){return 0;}
// uint8_t LiquidCrystal_I2C::init_bargraph(uint8_t graphtype){return 0;}
// void LiquidCrystal_I2C::draw_horizontal_graph(uint8_t row, uint8_t column, uint8_t len,  uint8_t pixel_col_end){}
// void LiquidCrystal_I2C::draw_vertical_graph(uint8_t row, uint8_t column, uint8_t len,  uint8_t pixel_row_end){}
// void LiquidCrystal_I2C::setContrast(uint8_t new_val){}
// #pragma GCC diagnostic pop

size_t HD44780_LiquidCrystal_I2C::printNumber(unsigned long n, uint8_t base)
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) {
    base = 10;
  }

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);

  return print(str);
}

size_t HD44780_LiquidCrystal_I2C::print(const char str[])
{
  if (str == NULL)
    return 0;

  const uint8_t *buffer = (const uint8_t *)str;
  size_t size = strlen(str);
  size_t n = 0;

  while (size--)
  {
    if (lcdWrite(*buffer++))
      n++;
    else
      break;
  }
  return n;
}

size_t HD44780_LiquidCrystal_I2C::print(char c)
{
  char str[]={c};
  return print(str);
}

size_t HD44780_LiquidCrystal_I2C::print(double number, int digits)
{
  size_t n = 0;

  if ((number != number) ? 1 : 0)
  {
    return print("nan");
  }
  if ((((((number) - (number)) != ((number) - (number))) ? 1 : 0) && !((number != number) ? 1 : 0)))
  {
    return print("inf");
  }
  if (number > 4294967040.0)
  {
    return print("ovf"); // constant determined empirically
  }
  if (number < -4294967040.0)
  {
    return print("ovf"); // constant determined empirically
  }

  // Handle negative numbers
  if (number < 0.0)
  {
    n += print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
  {
    rounding /= 10.0;
  }

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
  {
    n += print('.');
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)remainder;
    n += print(toPrint);
    remainder -= toPrint;
  }

  return n;
}

size_t HD44780_LiquidCrystal_I2C::print(int n, int base)
{
  return print((long) n, base);
}

size_t HD44780_LiquidCrystal_I2C::print(unsigned int n, int base)
{
  return print((unsigned long)n, base);
}

size_t HD44780_LiquidCrystal_I2C::print(long n, int base)
{
  if (base == 0) {
    return print(n);
  } else if (base == 10) {
    if (n < 0) {
      int t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  } else {
    return printNumber(n, base);
  }
}

size_t HD44780_LiquidCrystal_I2C::print(unsigned long n, int base)
{
  if (base == 0)
  {
    // return write(n);
  }
  else
  {
    char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    // prevent crash if called with base == 1
    if (base < 2)
    {
      base = 10;
    }

    do
    {
      unsigned long m = n;
      n /= base;
      char c = m - base * n;
      *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while (n);

    return print(str);
  }
}

/*********** mid level commands, for sending data/cmds */
inline size_t HD44780_LiquidCrystal_I2C::lcdWrite(uint8_t value)
{
  send(value, Rs);
  return 1;
}