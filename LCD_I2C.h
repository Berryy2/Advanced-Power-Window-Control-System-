#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

// Call once to initialize LCD
void LCD_Init(void);

// Clear LCD screen
void LCD_Clear(void);

// Set cursor to (row, col)
void LCD_SetCursor(uint8_t row, uint8_t col);

// Print string
void LCD_Print(const char *str);

// Print unsigned number
void LCD_PrintNum(uint32_t num);

// Control Backlight
void LCD_SetBacklight(uint8_t on);

#endif
