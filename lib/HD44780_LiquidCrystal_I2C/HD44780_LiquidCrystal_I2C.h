#ifndef HD44780_LIQUID_CRYSTAL_I2C_H
#define HD44780_LIQUID_CRYSTAL_I2C_H

#include "stm32f1xx_hal.h"
#include <string.h>
// #include <inttypes.h>
// #include <Print.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100 // Enable bit
#define Rw 0b00000010 // Read/Write bit
#define Rs 0b00000001 // Register select bit

extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;

class HD44780_LiquidCrystal_I2C
{
public:
    HD44780_LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows);
    void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);
    void clear();
    void home();
    void noDisplay();
    void display();
    // void noBlink();
    // void blink();
    // void noCursor();
    // void cursor();
    // void scrollDisplayLeft();
    // void scrollDisplayRight();
    // void printLeft();
    // void printRight();
    // void leftToRight();
    // void rightToLeft();
    // void shiftIncrement();
    // void shiftDecrement();
    void noBacklight();
    void backlight();
    // void autoscroll();
    // void noAutoscroll();
    // void createChar(uint8_t, uint8_t[]);
    // void createChar(uint8_t location, const char *charmap);
    // Example: 	const char bell[8] PROGMEM = {B00100,B01110,B01110,B01110,B11111,B00000,B00100,B00000};

    void setCursor(uint8_t, uint8_t);
    // #if defined(ARDUINO) && ARDUINO >= 100
    // virtual size_t write(uint8_t);
    // #else
    // virtual void write(uint8_t);
    // #endif
    void command(uint8_t);
    void init();
    //     void oled_init();

    // virtual size_t write(uint8_t) = 0;
    // size_t write(const char *str)
    // {
    //     if (str == NULL)
    //     {
    //         return 0;
    //     }
    //     return write((const uint8_t *)str, strlen(str));
    // }
    // virtual size_t write(const uint8_t *buffer, size_t size);
    // size_t write(const char *buffer, size_t size)
    // {
    //   return write((const uint8_t *)buffer, size);
    // }

    // // default to zero, meaning "a single write may block"
    // // should be overridden by subclasses with buffering
    // virtual int availableForWrite()
    // {
    //   return 0;
    // }

    // size_t print(const __FlashStringHelper *);
    // size_t print(const String &);
    size_t print(const char[]);
    size_t print(char);
    // size_t print(unsigned char, int = DEC);
    size_t print(int, int = 10);
    size_t print(unsigned int, int = 10);
    size_t print(long, int = 10);
    size_t print(unsigned long, int = 10);
    // size_t print(long long, int = DEC);
    // size_t print(unsigned long long, int = DEC);
    size_t print(double, int = 2);
    // size_t print(const Printable &);

    // size_t println(const __FlashStringHelper *);
    // size_t println(const String &s);
    // size_t println(const char[]);
    // size_t println(char);
    // size_t println(unsigned char, int = DEC);
    // size_t println(int, int = DEC);
    // size_t println(unsigned int, int = DEC);
    // size_t println(long, int = DEC);
    // size_t println(unsigned long, int = DEC);
    // size_t println(long long, int = DEC);
    // size_t println(unsigned long long, int = DEC);
    // size_t println(double, int = 2);
    // size_t println(const Printable &);
    // size_t println(void);

private:
    void init_priv();
    void send(uint8_t, uint8_t);
    void write4bits(uint8_t);
    void expanderWrite(uint8_t);
    void pulseEnable(uint8_t);
    inline size_t lcdWrite(uint8_t);
    size_t printNumber(unsigned long, uint8_t);
    uint8_t _Addr;
    uint8_t _displayfunction;
    uint8_t _displaycontrol;
    uint8_t _displaymode;
    uint8_t _numlines;
    bool _oled = false;
    uint8_t _cols;
    uint8_t _rows;
    uint8_t _backlightval;
};

#endif // LIQUID_CRYSTAL_I2C_H
