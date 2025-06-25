/* Includes ------------------------------------------------------------------*/
#include "rtc_utils.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/**
 * @brief Chuyển đổi số thập phân sang BCD.
 */
uint8_t My_RTC_DecToBcd(uint8_t val)
{
    return (uint8_t)(((val / 10) << 4) | (val % 10));
}

/**
 * @brief Chuyển đổi số BCD sang thập phân.
 */
uint8_t My_RTC_BcdToDec(uint8_t val)
{
    return (uint8_t)(((val >> 4) * 10) + (val & 0x0F));
}

/**
 * @brief Cài đặt thời gian và ngày tháng cho RTC.
 */
HAL_StatusTypeDef My_RTC_SetDateTime(RTC_HandleTypeDef *hrtc_ptr,
                                     uint8_t hours_bcd, uint8_t minutes_bcd, uint8_t seconds_bcd,
                                     uint8_t day_bcd, uint8_t month_bcd, uint8_t year_bcd, uint8_t weekday)
{
    RTC_TimeTypeDef sTimeSet = {0};
    RTC_DateTypeDef sDateSet = {0};
    HAL_StatusTypeDef time_status, date_status;

    sTimeSet.Hours = hours_bcd;
    sTimeSet.Minutes = minutes_bcd;
    sTimeSet.Seconds = seconds_bcd;
    sTimeSet.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTimeSet.StoreOperation = RTC_STOREOPERATION_RESET;
    time_status = HAL_RTC_SetTime(hrtc_ptr, &sTimeSet, RTC_FORMAT_BCD);
    if (time_status != HAL_OK)
    {
        return time_status;
    }

    sDateSet.WeekDay = weekday;
    sDateSet.Month = month_bcd;
    sDateSet.Date = day_bcd;
    sDateSet.Year = year_bcd;
    date_status = HAL_RTC_SetDate(hrtc_ptr, &sDateSet, RTC_FORMAT_BCD);

    return date_status;
}

/**
 * @brief Lấy thời gian và ngày tháng hiện tại từ RTC.
 */
HAL_StatusTypeDef My_RTC_GetDateTime(RTC_HandleTypeDef *hrtc_ptr, My_RTC_DateTime_t *dateTimeDest)
{
    HAL_StatusTypeDef status;
    status = HAL_RTC_GetTime(hrtc_ptr, &(dateTimeDest->time), RTC_FORMAT_BCD);
    if (status != HAL_OK)
    {
        return status;
    }
    status = HAL_RTC_GetDate(hrtc_ptr, &(dateTimeDest->date), RTC_FORMAT_BCD);
    return status;
}

void My_RTC_InitAndSet(RTC_HandleTypeDef *hrtc_ptr)
{

    if (HAL_RTCEx_BKUPRead(hrtc_ptr, RTC_BKP_DR0) != RTC_INIT_MAGIC_NUMBER)
    {
        printf("RTC: First time setup or Vbat lost. Setting default time...\r\n");

        uint8_t default_hours = My_RTC_DecToBcd(12);
        uint8_t default_minutes = My_RTC_DecToBcd(0);
        uint8_t default_seconds = My_RTC_DecToBcd(0);
        uint8_t default_day = My_RTC_DecToBcd(1);
        uint8_t default_month = RTC_MONTH_JANUARY;
        uint8_t default_year = My_RTC_DecToBcd(24);
        uint8_t default_weekday = RTC_WEEKDAY_MONDAY;

        if (My_RTC_SetDateTime(hrtc_ptr, default_hours, default_minutes, default_seconds,
                               default_day, default_month, default_year, default_weekday) == HAL_OK)
        {
            printf("RTC: Default time set successfully.\r\n");
            HAL_PWR_EnableBkUpAccess();
            HAL_RTCEx_BKUPWrite(hrtc_ptr, RTC_BKP_DR0, RTC_INIT_MAGIC_NUMBER);
            HAL_PWR_DisableBkUpAccess();
        }
        else
        {
            printf("RTC: Error setting default time.\r\n");
        }
    }
    else
    {
        printf("RTC: Already configured.\r\n");
    }
}