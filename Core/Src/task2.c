/* Includes ------------------------------------------------------------------*/
#include "task2.h"
#include "fram_i2c.h"
#include "string.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;
extern RTC_HandleTypeDef hrtc;

/**
 * @brief Khởi tạo cho Task 2.
 */
void Task2_Init(void)
{

}

/**
 * @brief Thực hiện một lần nháy (toggle) LED và delay cho Task 2.
 */
void Task2_LedBlink(GPIO_TypeDef *ledPort, uint16_t ledPin, uint32_t blinkSpeed_ms)
{
  HAL_GPIO_TogglePin(ledPort, ledPin);
  osDelay(blinkSpeed_ms);
}

float Read_Internal_Temperature(void)
{
  uint32_t adc_value = 0;
  float temperature_celsius = 0.0f;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
  {
    adc_value = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  float vsense_mv = ((float)adc_value * ADC_VREF_MV) / ADC_MAX_VALUE;
  temperature_celsius = (vsense_mv - TEMP_SENSOR_V25_MV) / TEMP_SENSOR_AVG_SLOPE_MV_PER_C + 25.0f;

  return temperature_celsius;
}

HAL_StatusTypeDef SaveTempLogToFRAM(I2C_HandleTypeDef *hi2c, uint16_t addr, TemperatureLog_t *log)
{
  return FRAM_WriteBytes(hi2c, addr, (uint8_t *)log, sizeof(TemperatureLog_t));
}

HAL_StatusTypeDef ReadTempLogFromFRAM(I2C_HandleTypeDef *hi2c, uint16_t addr, TemperatureLog_t *log)
{
  return FRAM_ReadBytes(hi2c, addr, (uint8_t *)log, sizeof(TemperatureLog_t));
}

/**
 * @brief Kiểm tra nút User Button và ghi nhiệt độ vào FRAM ngay lập tức khi nút được nhấn
 * @param previous_state Con trỏ đến biến lưu trạng thái nút trước đó
 * @retval None
 */
void CheckUserButtonAndSaveTemp(GPIO_PinState *previous_state)
{
  GPIO_PinState current_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

  if (current_button_state == GPIO_PIN_SET && *previous_state == GPIO_PIN_RESET)
  {
    TemperatureLog_t log;
    log.temperature = (int)Read_Internal_Temperature();

    My_RTC_GetDateTime(&hrtc, &log.dateTime);

    SaveTempLogToFRAM(&hi2c2, USER_TEMP_ADDR, &log);
  }

  *previous_state = current_button_state;
}
