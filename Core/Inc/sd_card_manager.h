#ifndef SD_CARD_MANAGER_H
#define SD_CARD_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

#define FILE_NAME "stm32.txt"
#include <stdint.h>

    /**
     * @brief  Khởi tạo SD Card và file system
     * @retval uint8_t: 1 nếu thành công, 0 nếu thất bại
     */
    uint8_t SD_Init(void);

    /**
     * @brief  Tạo và ghi file với nội dung cho trước
     * @param  filename: Tên file cần tạo/ghi
     * @param  content: Nội dung cần ghi vào file
     * @param  append: 1 = thêm vào cuối file, 0 = ghi đè file
     * @retval uint8_t: 1 nếu thành công, 0 nếu thất bại
     */
    uint8_t SD_WriteFile(const char *filename, const char *content, uint8_t append);

    /**
     * @brief  Tạo file team9.txt với nội dung về nhóm
     * @retval uint8_t: 1 nếu thành công, 0 nếu thất bại
     */
    uint8_t SD_CreateTeamFile(void);

    /**
     * @brief  Đọc nội dung từ file
     * @param  filename: Tên file cần đọc
     * @param  buffer: Buffer để lưu dữ liệu đọc được
     * @param  buffer_size: Kích thước của buffer
     * @retval int: Số byte đọc được, -1 nếu có lỗi
     */
    int SD_ReadFile(const char *filename, char *buffer, uint32_t buffer_size);

    /**
     * @brief  Đọc file team9.txt và lưu vào buffer được cung cấp
     * @param  buffer: Buffer để lưu dữ liệu đọc được
     * @param  buffer_size: Kích thước buffer
     * @retval int: Số byte đọc được, -1 nếu có lỗi
     */
    int SD_ReadTeamFile(char *buffer, uint32_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_MANAGER_H */
