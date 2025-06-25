/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_TASK2_H_ // Đổi tên include guard thành INC_TASK2_H_
#define INC_TASK2_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h" // Hoặc include cần thiết khác
#include "rtc_utils.h"     // Để dùng My_RTC_DateTime_t

typedef struct
{
    int temperature;
    My_RTC_DateTime_t dateTime;
} TemperatureLog_t;
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

#define BLINK_SPEED_200_MS 200
#define BLINK_SPEED_500_MS 500
#define BLINK_SPEED_1000_MS 1000

#define TEMP_SENSOR_AVG_SLOPE_MV_PER_C 2.5f // mV/°C (Typ. 2.5 mV/°C)
#define TEMP_SENSOR_V25_MV 760.0f           // mV (Typ. 760 mV at 25°C)
#define ADC_VREF_MV 3300.0f                 // Điện áp tham chiếu ADC (mV), thường là VDD = 3.3V
#define ADC_MAX_VALUE 4095.0f               // (2^12 - 1) cho ADC 12-bit

#define GROUP_NUMBER 9 // Số nhóm của bạn

#define USER_TEMP_ADDR 0x0100 // Địa chỉ cố định trong FRAM để lưu nhiệt độ

/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Khởi tạo cho Task 2 (hiện tại liên quan đến LED).
 * @note Có thể để trống nếu cấu hình GPIO đã đủ trong MX_GPIO_Init().
 * @retval None
 */
void Task2_Init(void); // Đổi tên hàm Init (tùy chọn, nhưng rõ ràng hơn)

/**
 * @brief Thực hiện một lần nháy (toggle) LED và delay cho Task 2.
 * @note Để nhấp nháy liên tục, gọi hàm này trong vòng lặp.
 *       Yêu cầu chân GPIO đã được cấu hình là Output Push-Pull.
 * @param ledPort: Con trỏ tới port GPIO (ví dụ: GPIOB).
 * @param ledPin: Số hiệu pin (ví dụ: GPIO_PIN_2).
 * @param blinkSpeed_ms: Thời gian LED giữ một trạng thái (sáng hoặc tắt) tính bằng ms.
 * @retval None
 */
void Task2_LedBlink(GPIO_TypeDef *ledPort, uint16_t ledPin, uint32_t blinkSpeed_ms); // Đổi tên hàm (tùy chọn)

/**
 * @brief Đọc nhiệt độ từ cảm biến nội bộ.
 * @note Sử dụng ADC1 để đọc giá trị nhiệt độ.
 * @retval Nhiệt độ tính bằng độ C.
 */
float Read_Internal_Temperature(void);

/**
 * @brief Kiểm tra nút User Button và ghi nhiệt độ vào FRAM ngay lập tức khi
 * nút được nhấn
 * @param previous_state Con trỏ đến biến lưu trạng thái nút trước đó
 *  @retval None
 */
HAL_StatusTypeDef SaveTempLogToFRAM(I2C_HandleTypeDef *hi2c, uint16_t addr, TemperatureLog_t *log);

/**
 * @brief Đọc nhiệt độ từ FRAM
 * @param hi2c Con trỏ đến cấu trúc I2C_HandleTypeDef
 * @param addr Địa chỉ trong FRAM để đọc
 * @param log Con trỏ đến cấu trúc TempLog_t để lưu kết quả
 * @retval HAL_StatusTypeDef Trạng thái của HAL (HAL_OK nếu thành công)
 */
HAL_StatusTypeDef ReadTempLogFromFRAM(I2C_HandleTypeDef *hi2c, uint16_t addr, TemperatureLog_t *log);

/**
 * @brief Kiểm tra nút User Button và ghi nhiệt độ vào FRAM ngay lập tức khi
 * nút được nhấn
 * @param previous_state Con trỏ đến biến lưu trạng thái nút trước đó
 * @retval None
 */
void CheckUserButtonAndSaveTemp(GPIO_PinState *previous_state);

#endif /* INC_TASK2_H_ */
