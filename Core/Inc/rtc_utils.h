/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_RTC_UTILS_H_
#define INC_RTC_UTILS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h" // Để có RTC_HandleTypeDef, RTC_TimeTypeDef, RTC_DateTypeDef

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
} My_RTC_DateTime_t;

/* Exported constants --------------------------------------------------------*/
#define RTC_INIT_MAGIC_NUMBER 0x4321 // Một số tùy chọn để đánh dấu RTC đã init

/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Khởi tạo và cài đặt RTC.
 * @note  Hàm này sẽ kiểm tra xem RTC đã được cài đặt thời gian trước đó chưa (qua Backup Register).
 *        Nếu chưa, nó sẽ cài đặt thời gian mặc định.
 *        Cần được gọi sau khi MX_RTC_Init() đã được gọi.
 * @param  hrtc_ptr: Con trỏ tới RTC_HandleTypeDef.
 * @retval None
 */
void My_RTC_InitAndSet(RTC_HandleTypeDef *hrtc_ptr);

/**
 * @brief Cài đặt thời gian và ngày tháng cho RTC một cách thủ công.
 * @note  Hàm này ghi đè thời gian hiện tại của RTC.
 * @param  hrtc_ptr: Con trỏ tới RTC_HandleTypeDef.
 * @param  hours_bcd: Giờ (định dạng BCD, ví dụ 0x10 cho 10 giờ).
 * @param  minutes_bcd: Phút (định dạng BCD).
 * @param  seconds_bcd: Giây (định dạng BCD).
 * @param  day_bcd: Ngày trong tháng (định dạng BCD).
 * @param  month_bcd: Tháng (sử dụng hằng số RTC_MONTH_x của HAL hoặc giá trị BCD).
 * @param  year_bcd: Năm (2 số cuối, định dạng BCD, ví dụ 0x24 cho 2024).
 * @param  weekday: Ngày trong tuần (sử dụng hằng số RTC_WEEKDAY_x của HAL).
 * @retval HAL_StatusTypeDef: Trạng thái của HAL (HAL_OK nếu thành công).
 */
HAL_StatusTypeDef My_RTC_SetDateTime(RTC_HandleTypeDef *hrtc_ptr,
                                     uint8_t hours_bcd, uint8_t minutes_bcd, uint8_t seconds_bcd,
                                     uint8_t day_bcd, uint8_t month_bcd, uint8_t year_bcd, uint8_t weekday);

/**
 * @brief Lấy thời gian và ngày tháng hiện tại từ RTC.
 * @param  hrtc_ptr: Con trỏ tới RTC_HandleTypeDef.
 * @param  dateTimeDest: Con trỏ tới struct My_RTC_DateTime_t để lưu kết quả.
 * @retval HAL_StatusTypeDef: Trạng thái của HAL (HAL_OK nếu thành công).
 */
HAL_StatusTypeDef My_RTC_GetDateTime(RTC_HandleTypeDef *hrtc_ptr, My_RTC_DateTime_t *dateTimeDest);

/**
 * @brief Chuyển đổi số thập phân sang BCD.
 * @param  val: Giá trị thập phân.
 * @retval Giá trị BCD.
 */
uint8_t My_RTC_DecToBcd(uint8_t val);

/**
 * @brief Chuyển đổi số BCD sang thập phân.
 * @param  val: Giá trị BCD.
 * @retval Giá trị thập phân.
 */
uint8_t My_RTC_BcdToDec(uint8_t val);

#endif