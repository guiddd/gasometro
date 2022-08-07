#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

void setup() {
  /*
    BMP180
    MQ7
    MQ5
    Buzzer
    Temt6000
    ----------Display Oled SPI
  */
  lcd_i2c.init(); // initialize the lcd
  lcd_i2c.backlight();

  lcd_i2c.setCursor(0, 0);      // move cursor to   (0, 0)
  lcd_i2c.print("Hello");       // print message at (0, 0)
  lcd_i2c.setCursor(2, 1);      // move cursor to   (2, 1)
  lcd_i2c.print("esp32io.com"); // print message at (2, 1)
}

void loop() {
}
