#include "TM4C123.h"
#include "LCD_I2C.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

// I2C address of LCD (change to 0x3F if needed)
#define LCD_ADDR 0x27
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BACKLIGHT 0x08

extern uint8_t lcd_backlight; // Global flag to track backlight status
uint8_t lcd_backlight = 1;

static void I2C0_Init(void);
static void I2C0_WriteByte(uint8_t addr, uint8_t data);
static void LCD_Write4Bits(uint8_t data);
static void LCD_Cmd(uint8_t cmd);
static void LCD_Data(uint8_t data);
static void LCD_WriteNibble(uint8_t nibble, bool is_data);

void LCD_Init(void) {
		lcd_backlight = 0x0;											// Start with backlight off
    I2C0_Init();															// Set up I2C communication
		
		// Special initialization sequence (4-bit mode)
    for(volatile int i = 0; i < 50000; i++); // ~50ms delay
    LCD_WriteNibble(0x03 << 4, false);
    for(volatile int i = 0; i < 5000; i++);  // ~5ms delay
    LCD_WriteNibble(0x03 << 4, false);
    for(volatile int i = 0; i < 5000; i++);  // ~5ms delay
    LCD_WriteNibble(0x03 << 4, false);
    for(volatile int i = 0; i < 1000; i++);  // ~1ms delay
    LCD_WriteNibble(0x02 << 4, false);
		
		// Configure LCD settings
    LCD_Cmd(0x28);	// 4-bit mode, 2 lines, 5x8 font
    LCD_Cmd(0x0C);	// Display on, cursor off, blink off
    LCD_Cmd(0x06);	// Increment cursor, no display shift
    LCD_Cmd(0x01);	// Clear display
    for(volatile int i = 0; i < 2000; i++);  // ~2ms delay
}

// Wipes the entire screen
void LCD_Clear(void) {
    LCD_Cmd(0x01);
    for(volatile int i = 0; i < 2000; i++);  // ~2ms delay
}

// Moves cursor to specific position
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40};
    LCD_Cmd(0x80 | (col + row_offsets[row]));
}

// Shows text at current cursor
void LCD_Print(const char *str) {
    while (*str) {
        LCD_Data(*str++);
    }
}

// Shows a number
void LCD_PrintNum(uint32_t num) {
    char buf[12];
    sprintf(buf, "%lu", num);
    LCD_Print(buf);
}

// Controls backlight
void LCD_SetBacklight(uint8_t on) {
    lcd_backlight = on ? 0x08 : 0x00;
}

// Low-level I2C + LCD
static void I2C0_Init(void) {
		// 1. Turn on I2C and GPIO clocks
    SYSCTL->RCGCI2C |= 1;										// Enable I2C0 clock
    SYSCTL->RCGCGPIO |= (1U << 1);					// Enable GPIO Port B clock
    while (!(SYSCTL->PRGPIO & (1U << 1)));	// Wait until ready
		
		// 2. Configure PB2 (SCL) and PB3 (SDA)
    GPIOB->AFSEL |= (1U << 2) | (1U << 3);	// Enable alternate function
    GPIOB->ODR |= (1U << 3);								// SDA open-drain
    GPIOB->DEN |= (1U << 2) | (1U << 3);		// Digital enable
    GPIOB->PCTL |= (3U << 8) | (3U << 12);	// Select I2C function

		// 3. Configure I2C module
    I2C0->MCR = 0x10;												// Enable I2C master
    I2C0->MTPR = 39;												// Set speed to ~50kHz
}

static void I2C0_WriteByte(uint8_t addr, uint8_t data) {
    I2C0->MSA = (addr << 1);
    I2C0->MDR = data;
    I2C0->MCS = 3;
    while (I2C0->MCS & 1);
}

static void LCD_PulseEnable(uint8_t data) {
    I2C0_WriteByte(LCD_ADDR, data | LCD_EN | lcd_backlight);
    I2C0_WriteByte(LCD_ADDR, data & ~LCD_EN | lcd_backlight);
}

static void LCD_Write4Bits(uint8_t data) {
    I2C0_WriteByte(LCD_ADDR, data | lcd_backlight);
    LCD_PulseEnable(data);
}

static void LCD_WriteNibble(uint8_t nibble, bool is_data) {
    uint8_t data = (is_data ? LCD_RS : 0) | (nibble & 0xF0);
    LCD_Write4Bits(data);
}

static void LCD_Cmd(uint8_t cmd) {
    LCD_WriteNibble(cmd & 0xF0, false);
    LCD_WriteNibble((cmd << 4) & 0xF0, false);
}

static void LCD_Data(uint8_t data) {
    LCD_WriteNibble(data & 0xF0, true);
    LCD_WriteNibble((data << 4) & 0xF0, true);
}
