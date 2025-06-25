#ifndef __MANUAL_LCD_H
#define __MANUAL_LCD_H

#include <stdint.h>        // Ensure standard types are defined
#include "stm32f4xx_hal.h" // Moved up to ensure types are defined

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h" // To access GPIO definitions and SPI handle

// Define LCD screen dimensions
#define LCD_WIDTH 240
#define LCD_HEIGHT 320

// Define some basic colors (RGB565 format)
#define COLOR_BLACK 0x0000
#define COLOR_BLUE 0x001F
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_CYAN 0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW 0xFFE0
#define COLOR_WHITE 0xFFFF

// Define layout constants
#define MARGIN 10
#define SPACING_BETWEEN_BOXES 10
#define TASK_BOX_W_CALC ((LCD_WIDTH - 2 * MARGIN - SPACING_BETWEEN_BOXES) / 2)
#define TASK_BOX_H 70
#define INFO_BOX_Y 10
#define INFO_BOX_H 30
#define INFO_BOX_W 150 // Standardized width for the info box
#define BACK_BUTTON_H 45
#define BACK_BUTTON_W 90 // Standardized width for the back button as per main.c
#define INFO_TO_TASK_SPACING 20
#define TASK_TO_BACK_BUTTON_SPACING 20

    // Exported functions
    void Manual_LCD_Init(void);
    void Manual_LCD_Clear(uint16_t color);
    void Manual_LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
    void Manual_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
    void Manual_LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
    void Manual_LCD_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
    void Manual_LCD_FillRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
    void Manual_LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t charColor, uint16_t bgColor, uint8_t size);
    void Manual_LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t charColor, uint16_t bgColor, uint8_t size);
    void Manual_LCD_DrawLayout(void); // As used in your main.c

    // New functions for updating layout elements
    void Manual_LCD_UpdateInfoText(const char *new_text);
    void Manual_LCD_RefillTaskBox(const char *task_name, uint16_t new_fill_color);

    // Hàm vẽ màn hình danh sách thành viên
    void Manual_LCD_DrawMembersLayout(void);

#ifdef __cplusplus
}
#endif

#endif /* __MANUAL_LCD_H */