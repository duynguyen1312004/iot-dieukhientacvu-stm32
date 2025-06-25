#ifndef __MANUAL_TOUCH_H
#define __MANUAL_TOUCH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "manual_lcd.h" // For Coordinate, Matrix, LCD_WIDTH, LCD_HEIGHT, drawing functions

    // From original Touchpanel.h
    typedef struct POINT
    {
        uint16_t x;
        uint16_t y;
    } Coordinate;

    typedef struct Matrix
    {
        long double An, Bn, Cn, Dn, En, Fn, Divider;
    } Matrix;

// ADS7846/XPT2046 Control Bytes
// Using original defines first. If X/Y are swapped, these need to be flipped.
// Standard: X_CMD = 0xD0, Y_CMD = 0x90
#define TP_CMD_READ_Y 0x90 // Original CHX
#define TP_CMD_READ_X 0xD0 // Original CHY

// Touch Raw Threshold
#define TOUCH_AVG_SAMPLES 9
#define TOUCH_RAW_THRESHOLD 35 // Increased from 20 to 35

    extern Matrix touchMatrix; // Calibration matrix, to be accessible by main

    void Manual_Touch_Init(SPI_HandleTypeDef *hspi_touch);
    uint8_t Manual_Touch_Pressed(void);
    uint8_t Manual_Touch_GetRawPoint(Coordinate *rawPoint);
    void Manual_Touch_Calibrate(uint16_t charColor, uint16_t bkColor);
    uint8_t Manual_Touch_ApplyCalibration(Coordinate *displayPoint, const Coordinate *screenPoint);
    void Manual_Touch_SetFixedCalibration(const Coordinate CalDisplay[], const Coordinate CalScreen[]);

#ifdef __cplusplus
}
#endif

#endif /* __MANUAL_TOUCH_H */
