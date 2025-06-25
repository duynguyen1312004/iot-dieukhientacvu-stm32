#include "manual_lcd.h"
#include "font.h" // You'll need to create this file or provide font data
#include <string.h>

// External SPI handle defined in main.c
extern SPI_HandleTypeDef hspi1;

// Low-level SPI communication functions
static void LCD_SPI_Send(uint8_t data)
{
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
}

static void LCD_Write_Cmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET); // DC low for command
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS low to select
    LCD_SPI_Send(cmd);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS high to deselect
}

static void LCD_Write_Data(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);   // DC high for data
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS low to select
    LCD_SPI_Send(data);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS high to deselect
}

static void LCD_Write_Data16(uint16_t data)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);   // DC high for data
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS low to select
    uint8_t bytes[2];
    bytes[0] = (data >> 8) & 0xFF;
    bytes[1] = data & 0xFF;
    HAL_SPI_Transmit(&hspi1, bytes, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS high to deselect
}

// Set drawing window
static void LCD_Set_Address_Window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_Write_Cmd(0x2A); // Column Address Set
    LCD_Write_Data(x1 >> 8);
    LCD_Write_Data(x1 & 0xFF);
    LCD_Write_Data(x2 >> 8);
    LCD_Write_Data(x2 & 0xFF);

    LCD_Write_Cmd(0x2B); // Page Address Set
    LCD_Write_Data(y1 >> 8);
    LCD_Write_Data(y1 & 0xFF);
    LCD_Write_Data(y2 >> 8);
    LCD_Write_Data(y2 & 0xFF);

    LCD_Write_Cmd(0x2C); // Memory Write
}

void Manual_LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int16_t x = r;
    int16_t y = 0;
    int16_t err = 0;

    while (x >= y)
    {
        Manual_LCD_DrawPixel(x0 + x, y0 + y, color);
        Manual_LCD_DrawPixel(x0 + y, y0 + x, color);
        Manual_LCD_DrawPixel(x0 - y, y0 + x, color);
        Manual_LCD_DrawPixel(x0 - x, y0 + y, color);
        Manual_LCD_DrawPixel(x0 - x, y0 - y, color);
        Manual_LCD_DrawPixel(x0 - y, y0 - x, color);
        Manual_LCD_DrawPixel(x0 + y, y0 - x, color);
        Manual_LCD_DrawPixel(x0 + x, y0 - y, color);

        if (err <= 0)
        {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0)
        {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

void Manual_LCD_Init(void)
{

    // Hardware Reset
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    // Backlight ON
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // Initialization sequence (Example for ILI9341 - ADJUST FOR YOUR LCD)
    LCD_Write_Cmd(0xCF); // Power Control B
    LCD_Write_Data(0x00);
    LCD_Write_Data(0xC1);
    LCD_Write_Data(0x30);

    LCD_Write_Cmd(0xED); // Power on sequence control
    LCD_Write_Data(0x64);
    LCD_Write_Data(0x03);
    LCD_Write_Data(0x12);
    LCD_Write_Data(0x81);

    LCD_Write_Cmd(0xE8); // Driver timing control A
    LCD_Write_Data(0x85);
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x78);

    LCD_Write_Cmd(0xCB); // Power control A
    LCD_Write_Data(0x39);
    LCD_Write_Data(0x2C);
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x34);
    LCD_Write_Data(0x02);

    LCD_Write_Cmd(0xF7); // Pump ratio control
    LCD_Write_Data(0x20);

    LCD_Write_Cmd(0xEA); // Driver timing control B
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x00);

    LCD_Write_Cmd(0xC0);  // Power Control 1
    LCD_Write_Data(0x23); // VRH[5:0]

    LCD_Write_Cmd(0xC1);  // Power Control 2
    LCD_Write_Data(0x10); // SAP[2:0];BT[3:0]

    LCD_Write_Cmd(0xC5);  // VCOM Control 1
    LCD_Write_Data(0x3E); // Contrast
    LCD_Write_Data(0x28);

    LCD_Write_Cmd(0xC7);  // VCOM Control 2
    LCD_Write_Data(0x86); // --

    LCD_Write_Cmd(0x36); // Memory Access Control
    // LCD_Write_Data(0x08); // Previous value
    LCD_Write_Data(0x28); // Try this: MV=1, MY=0, MX=0, BGR=1 (Landscape, X:T-B, Y:L-R)

    LCD_Write_Cmd(0x3A);  // Pixel Format Set
    LCD_Write_Data(0x55); // 16 bits/pixel

    LCD_Write_Cmd(0xB1); // Frame Rate Control (In Normal Mode/Full Colors)
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x18);

    LCD_Write_Cmd(0xB6); // Display Function Control
    LCD_Write_Data(0x08);
    LCD_Write_Data(0x82);
    LCD_Write_Data(0x27);

    LCD_Write_Cmd(0xF2);  // Enable 3G
    LCD_Write_Data(0x00); // 3Gamma Function Disable

    LCD_Write_Cmd(0x26);  // Gamma Set
    LCD_Write_Data(0x01); // Gamma curve selected

    // ... (Positive and Negative Gamma Correction remain the same) ...
    LCD_Write_Cmd(0xE0); // Positive Gamma Correction
    LCD_Write_Data(0x0F);
    LCD_Write_Data(0x31);
    LCD_Write_Data(0x2B);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x0E);
    LCD_Write_Data(0x08);
    LCD_Write_Data(0x4E);
    LCD_Write_Data(0xF1);
    LCD_Write_Data(0x37);
    LCD_Write_Data(0x07);
    LCD_Write_Data(0x10);
    LCD_Write_Data(0x03);
    LCD_Write_Data(0x0E);
    LCD_Write_Data(0x09);
    LCD_Write_Data(0x00);

    LCD_Write_Cmd(0xE1); // Negative Gamma Correction
    LCD_Write_Data(0x00);
    LCD_Write_Data(0x0E);
    LCD_Write_Data(0x14);
    LCD_Write_Data(0x03);
    LCD_Write_Data(0x11);
    LCD_Write_Data(0x07);
    LCD_Write_Data(0x31);
    LCD_Write_Data(0xC1);
    LCD_Write_Data(0x48);
    LCD_Write_Data(0x08);
    LCD_Write_Data(0x0F);
    LCD_Write_Data(0x0C);
    LCD_Write_Data(0x31);
    LCD_Write_Data(0x36);
    LCD_Write_Data(0x0F);

    LCD_Write_Cmd(0x11); // Exit Sleep
    HAL_Delay(120);
    LCD_Write_Cmd(0x29); // Display ON

    // Manual_LCD_Clear(COLOR_BLACK);

    // Enable touch interrupt
}

void Manual_LCD_Clear(uint16_t color)
{
    LCD_Set_Address_Window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);   // DC high for data
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS low to select

    uint8_t color_bytes[2] = {(color >> 8) & 0xFF, color & 0xFF};
    for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * LCD_HEIGHT; i++)
    {
        HAL_SPI_Transmit(&hspi1, color_bytes, 2, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS high to deselect
}

void Manual_LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
        return;
    LCD_Set_Address_Window(x, y, x, y);
    LCD_Write_Data16(color);
}

void Manual_LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int16_t steep = abs(y2 - y1) > abs(x2 - x1);
    if (steep)
    {
        uint16_t temp;
        temp = x1;
        x1 = y1;
        y1 = temp;
        temp = x2;
        x2 = y2;
        y2 = temp;
    }
    if (x1 > x2)
    {
        uint16_t temp;
        temp = x1;
        x1 = x2;
        x2 = temp;
        temp = y1;
        y1 = y2;
        y2 = temp;
    }
    int16_t dx = x2 - x1;
    int16_t dy = abs(y2 - y1);
    int16_t err = dx / 2;
    int16_t ystep;
    if (y1 < y2)
        ystep = 1;
    else
        ystep = -1;

    for (; x1 <= x2; x1++)
    {
        if (steep)
            Manual_LCD_DrawPixel(y1, x1, color);
        else
            Manual_LCD_DrawPixel(x1, y1, color);
        err -= dy;
        if (err < 0)
        {
            y1 += ystep;
            err += dx;
        }
    }
}

void Manual_LCD_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT))
        return;
    if ((x + width - 1) >= LCD_WIDTH)
        width = LCD_WIDTH - x;
    if ((y + height - 1) >= LCD_HEIGHT)
        height = LCD_HEIGHT - y;

    Manual_LCD_DrawLine(x, y, x + width - 1, y, color);
    Manual_LCD_DrawLine(x, y + height - 1, x + width - 1, y + height - 1, color);
    Manual_LCD_DrawLine(x, y, x, y + height - 1, color);
    Manual_LCD_DrawLine(x + width - 1, y, x + width - 1, y + height - 1, color);
}

void Manual_LCD_FillRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT))
        return;
    if ((x + width) > LCD_WIDTH)
        width = LCD_WIDTH - x;
    if ((y + height) > LCD_HEIGHT)
        height = LCD_HEIGHT - y;

    LCD_Set_Address_Window(x, y, x + width - 1, y + height - 1);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);   // DC high for data
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS low to select

    uint8_t color_bytes[2] = {(color >> 8) & 0xFF, color & 0xFF};
    for (uint32_t i = 0; i < (uint32_t)width * height; i++)
    {
        HAL_SPI_Transmit(&hspi1, color_bytes, 2, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS high to deselect
}

// Basic font (5x7) - For a more complete solution, use a proper font library or a larger font array
// This is a very simplified character drawing.
// You should create a font.c/font.h with actual font data (e.g., extern const unsigned char font_5x7[][5];)
// For simplicity, this example will draw a fixed pattern for 'A' and 'B', etc.
// A proper implementation would look up character data in a font table.
// Placeholder for font data - you need to define this
extern const unsigned char Font5x7[][5]; // Example: needs to be defined in font.c

void Manual_LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t charColor, uint16_t bgColor, uint8_t size)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT || (x + 5 * size) > LCD_WIDTH || (y + 7 * size) > LCD_HEIGHT)
    {
        return; // Character won't fit
    }

    if (c < ' ' || c > '~')
        c = '?'; // Handle non-printable characters

    for (uint8_t i = 0; i < 5; i++)
    { // Width of char
        uint8_t line = Font5x7[c - ' '][i];
        for (uint8_t j = 0; j < 7; j++)
        { // Height of char
            if (line & 0x01)
            { // Check bit
                if (size == 1)
                {
                    Manual_LCD_DrawPixel(x + i, y + j, charColor);
                }
                else
                {
                    Manual_LCD_FillRectangle(x + i * size, y + j * size, size, size, charColor);
                }
            }
            else
            {
                if (size == 1)
                {
                    Manual_LCD_DrawPixel(x + i, y + j, bgColor);
                }
                else
                {
                    Manual_LCD_FillRectangle(x + i * size, y + j * size, size, size, bgColor);
                }
            }
            line >>= 1;
        }
    }
    // Draw a small gap for characters if size > 1 or if you want background fill for the gap
    if (size > 0)
    { // Fill the column after the character (for spacing)
        Manual_LCD_FillRectangle(x + 5 * size, y, size, 7 * size, bgColor);
    }
}

void Manual_LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t charColor, uint16_t bgColor, uint8_t size)
{
    while (*str)
    {
        Manual_LCD_DrawChar(x, y, *str, charColor, bgColor, size);
        x += 6 * size; // Advance X position (5 for char + 1 for spacing)
        if (x + 5 * size >= LCD_WIDTH)
        { // Simple wrap
            x = 0;
            y += 8 * size; // Advance Y position
        }
        if (y + 7 * size >= LCD_HEIGHT)
            break; // Out of screen
        str++;
    }
}

void Manual_LCD_DrawLayout(void)
{
    // Clear screen with blue background
    Manual_LCD_Clear(COLOR_BLUE);

    uint16_t text_color = COLOR_WHITE;
    uint16_t box_fill_color = COLOR_BLACK;
    uint16_t box_border_color = COLOR_WHITE;
    uint8_t font_size = 1;
    uint8_t char_height = 7; // For 5x7 font at size 1
    // uint8_t char_width_with_spacing = 6; // For 5x7 font at size 1

    // 1. Top Info Box: "Nhom: [nhom 01]"
    const char *info_text = "Nhom: 09";
    uint16_t info_text_x = MARGIN + 5;
    uint16_t info_text_y = INFO_BOX_Y + (INFO_BOX_H - char_height * font_size) / 2;

    Manual_LCD_FillRectangle(MARGIN, INFO_BOX_Y, INFO_BOX_W, INFO_BOX_H, box_fill_color);
    Manual_LCD_DrawRectangle(MARGIN, INFO_BOX_Y, INFO_BOX_W, INFO_BOX_H, box_border_color);
    Manual_LCD_DrawString(info_text_x, info_text_y, info_text, text_color, box_fill_color, font_size);

    // 2. Task Boxes Grid
    uint16_t start_y_tasks_row1 = INFO_BOX_Y + INFO_BOX_H + INFO_TO_TASK_SPACING;
    uint16_t text_task_x_padding = 10; // Left padding for text inside task boxes
    uint16_t text_task_y_offset = (TASK_BOX_H - char_height * font_size) / 2;

    // Task 02-1 (Top-Left)
    uint16_t task1_x = MARGIN;
    uint16_t task1_y = start_y_tasks_row1;
    const char *task1_text = "Task 02-1";
    uint16_t text_task1_x = task1_x + text_task_x_padding;
    uint16_t text_task1_y = task1_y + text_task_y_offset;
    Manual_LCD_FillRectangle(task1_x, task1_y, TASK_BOX_W_CALC, TASK_BOX_H, box_fill_color);
    Manual_LCD_DrawRectangle(task1_x, task1_y, TASK_BOX_W_CALC, TASK_BOX_H, box_border_color);
    Manual_LCD_DrawString(text_task1_x, text_task1_y, task1_text, text_color, box_fill_color, font_size);

    // Task 02-2 (Top-Right)
    uint16_t task2_x = task1_x + TASK_BOX_W_CALC + SPACING_BETWEEN_BOXES;
    uint16_t task2_y = start_y_tasks_row1;
    const char *task2_text = "Task 02-2";
    uint16_t text_task2_x = task2_x + text_task_x_padding;
    uint16_t text_task2_y = task2_y + text_task_y_offset;
    Manual_LCD_FillRectangle(task2_x, task2_y, TASK_BOX_W_CALC, TASK_BOX_H, box_fill_color);
    Manual_LCD_DrawRectangle(task2_x, task2_y, TASK_BOX_W_CALC, TASK_BOX_H, box_border_color);
    Manual_LCD_DrawString(text_task2_x, text_task2_y, task2_text, text_color, box_fill_color, font_size);

    // Second row of task boxes
    uint16_t start_y_tasks_row2 = start_y_tasks_row1 + TASK_BOX_H + SPACING_BETWEEN_BOXES;

    // Task 02-3 (Bottom-Left)
    uint16_t task3_x = MARGIN;
    uint16_t task3_y = start_y_tasks_row2;
    const char *task3_text = "Task 02-3";
    uint16_t text_task3_x = task3_x + text_task_x_padding;
    uint16_t text_task3_y = task3_y + text_task_y_offset;
    Manual_LCD_FillRectangle(task3_x, task3_y, TASK_BOX_W_CALC, TASK_BOX_H, box_fill_color);
    Manual_LCD_DrawRectangle(task3_x, task3_y, TASK_BOX_W_CALC, TASK_BOX_H, box_border_color);
    Manual_LCD_DrawString(text_task3_x, text_task3_y, task3_text, text_color, box_fill_color, font_size);

    // Task 02-4 (Bottom-Right)
    uint16_t task4_x = task1_x + TASK_BOX_W_CALC + SPACING_BETWEEN_BOXES;
    uint16_t task4_y = start_y_tasks_row2;
    const char *task4_text = "Task 02-4";
    uint16_t text_task4_x = task4_x + text_task_x_padding;
    uint16_t text_task4_y = task4_y + text_task_y_offset;
    Manual_LCD_FillRectangle(task4_x, task4_y, TASK_BOX_W_CALC, TASK_BOX_H, box_fill_color);
    Manual_LCD_DrawRectangle(task4_x, task4_y, TASK_BOX_W_CALC, TASK_BOX_H, box_border_color);
    Manual_LCD_DrawString(text_task4_x, text_task4_y, task4_text, text_color, box_fill_color, font_size);

    // 3. Back Button
    const char *back_button_text = "<<Back";
    uint16_t back_button_text_len = strlen(back_button_text);
    uint16_t back_button_text_pixel_width = back_button_text_len * 6 * font_size; // 6 is char width with spacing for 5x7 font

    uint16_t back_button_y = start_y_tasks_row2 + TASK_BOX_H + TASK_TO_BACK_BUTTON_SPACING;
    uint16_t back_button_x = MARGIN + TASK_BOX_W_CALC + SPACING_BETWEEN_BOXES;
    uint16_t text_button_x = back_button_x + (BACK_BUTTON_W - back_button_text_pixel_width) / 2;
    uint16_t text_button_y = back_button_y + (BACK_BUTTON_H - char_height * font_size) / 2;

    Manual_LCD_FillRectangle(back_button_x, back_button_y, BACK_BUTTON_W, BACK_BUTTON_H, box_fill_color);
    Manual_LCD_DrawRectangle(back_button_x, back_button_y, BACK_BUTTON_W, BACK_BUTTON_H, box_border_color);
    Manual_LCD_DrawString(text_button_x, text_button_y, back_button_text, text_color, box_fill_color, font_size);
}

// New function to update the info text
void Manual_LCD_UpdateInfoText(const char *new_text)
{
    uint8_t font_size = 1;
    uint8_t char_height = 7; // For 5x7 font at size 1

    uint16_t text_color = COLOR_WHITE;     // Original text color from DrawLayout
    uint16_t box_fill_color = COLOR_BLACK; // Original box fill color from DrawLayout

    uint16_t info_text_x = MARGIN + 5;
    uint16_t info_text_y = INFO_BOX_Y + (INFO_BOX_H - char_height * font_size) / 2;

    // Clear the inner part of the info box where text resides
    // The border is 1px, so we fill from x+1, y+1 with width-2, height-2
    Manual_LCD_FillRectangle(MARGIN + 1, INFO_BOX_Y + 1, INFO_BOX_W - 2, INFO_BOX_H - 2, box_fill_color);

    // Draw the new string with original text color and original box fill as background
    Manual_LCD_DrawString(info_text_x, info_text_y, new_text, text_color, box_fill_color, font_size);
}

// New function to refill a task box and redraw its text
void Manual_LCD_RefillTaskBox(const char *task_name, uint16_t new_fill_color)
{
    uint16_t original_text_color = COLOR_WHITE;
    uint16_t original_box_border_color = COLOR_WHITE;
    uint8_t font_size = 1;
    uint8_t char_height = 7; // For 5x7 font at size 1

    uint16_t text_task_x_padding = 10;
    uint16_t text_task_y_offset = (TASK_BOX_H - char_height * font_size) / 2;

    uint16_t current_task_x = 0, current_task_y = 0;
    uint16_t text_draw_x = 0, text_draw_y = 0;
    const char *original_task_text_content = "";

    if (strcmp(task_name, "Task 02-1") == 0)
    {
        current_task_x = MARGIN;
        current_task_y = INFO_BOX_Y + INFO_BOX_H + INFO_TO_TASK_SPACING;
        original_task_text_content = "Task 02-1";
    }
    else if (strcmp(task_name, "Task 02-2") == 0)
    {
        current_task_x = MARGIN + TASK_BOX_W_CALC + SPACING_BETWEEN_BOXES;
        current_task_y = INFO_BOX_Y + INFO_BOX_H + INFO_TO_TASK_SPACING;
        original_task_text_content = "Task 02-2";
    }
    else if (strcmp(task_name, "Task 02-3") == 0)
    {
        current_task_x = MARGIN;
        current_task_y = INFO_BOX_Y + INFO_BOX_H + INFO_TO_TASK_SPACING + TASK_BOX_H + SPACING_BETWEEN_BOXES;
        original_task_text_content = "Task 02-3";
    }
    else if (strcmp(task_name, "Task 02-4") == 0)
    {
        current_task_x = MARGIN + TASK_BOX_W_CALC + SPACING_BETWEEN_BOXES;
        current_task_y = INFO_BOX_Y + INFO_BOX_H + INFO_TO_TASK_SPACING + TASK_BOX_H + SPACING_BETWEEN_BOXES;
        original_task_text_content = "Task 02-4";
    }
    else
    {
        return; // Unknown task name, do nothing
    }

    text_draw_x = current_task_x + text_task_x_padding;
    text_draw_y = current_task_y + text_task_y_offset;

    // 1. Refill the entire task box with the new color
    Manual_LCD_FillRectangle(current_task_x, current_task_y, TASK_BOX_W_CALC, TASK_BOX_H, new_fill_color);

    // 2. Redraw the border
    Manual_LCD_DrawRectangle(current_task_x, current_task_y, TASK_BOX_W_CALC, TASK_BOX_H, original_box_border_color);

    // 3. Redraw the original text using the original text color and the new_fill_color as background
    Manual_LCD_DrawString(text_draw_x, text_draw_y, original_task_text_content, original_text_color, new_fill_color, font_size);
}

void Manual_LCD_DrawMembersLayout(void)
{

    Manual_LCD_Clear(COLOR_CYAN);
    Manual_LCD_FillRectangle(10, 10, 220, 40, COLOR_WHITE);
    Manual_LCD_DrawRectangle(10, 10, 220, 40, COLOR_BLACK);
    Manual_LCD_DrawString(60, 20, "NHOM 09", COLOR_BLACK, COLOR_WHITE, 3);

    Manual_LCD_DrawString(60, 60, "DANH SACH THANH VIEN", COLOR_BLACK, COLOR_CYAN, 1);

    uint16_t table_x = 10;
    uint16_t table_y = 90;
    uint16_t table_w = 220;
    uint16_t row_h = 32;
    uint16_t col1_w = 140;
    uint16_t col2_w = 80;
    uint16_t num_rows = 6;

    Manual_LCD_DrawRectangle(table_x, table_y, table_w, row_h * num_rows, COLOR_BLACK);
    for (int i = 1; i < num_rows; ++i)
        Manual_LCD_DrawLine(table_x, table_y + i * row_h, table_x + table_w, table_y + i * row_h, COLOR_BLACK);
    Manual_LCD_DrawLine(table_x + col1_w, table_y, table_x + col1_w, table_y + row_h * num_rows, COLOR_BLACK);

    Manual_LCD_DrawString(table_x + 10, table_y + 10, "Ho Ten", COLOR_BLACK, COLOR_CYAN, 1);
    Manual_LCD_DrawString(table_x + col1_w + 10, table_y + 10, "MSSV", COLOR_BLACK, COLOR_CYAN, 1);

    Manual_LCD_DrawString(table_x + 10, table_y + row_h + 10, "Hong Quoc Bao", COLOR_BLACK, COLOR_CYAN, 1);
    Manual_LCD_DrawString(table_x + col1_w + 10, table_y + row_h + 10, "22900011", COLOR_BLACK, COLOR_CYAN, 1);

    Manual_LCD_DrawString(table_x + 10, table_y + 2 * row_h + 10, "Tran Van Diep", COLOR_BLACK, COLOR_CYAN, 1);
    Manual_LCD_DrawString(table_x + col1_w + 10, table_y + 2 * row_h + 10, "22200031", COLOR_BLACK, COLOR_CYAN, 1);

    Manual_LCD_DrawString(table_x + 10, table_y + 3 * row_h + 10, "Nguyen Huu Duy", COLOR_BLACK, COLOR_CYAN, 1);
    Manual_LCD_DrawString(table_x + col1_w + 10, table_y + 3 * row_h + 10, "22200042", COLOR_BLACK, COLOR_CYAN, 1);

    Manual_LCD_DrawString(table_x + 10, table_y + 4 * row_h + 10, "Le Ba Hien", COLOR_BLACK, COLOR_CYAN, 1);
    Manual_LCD_DrawString(table_x + col1_w + 10, table_y + 4 * row_h + 10, "22200057", COLOR_BLACK, COLOR_CYAN, 1);

    Manual_LCD_DrawString(table_x + 10, table_y + 5 * row_h + 10, "Nguyen Dai Minh Huy", COLOR_BLACK, COLOR_CYAN, 1);
    Manual_LCD_DrawString(table_x + col1_w + 10, table_y + 5 * row_h + 10, "22200073", COLOR_BLACK, COLOR_CYAN, 1);

}
