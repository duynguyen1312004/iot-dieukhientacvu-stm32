# Đồ án Hệ thống nhúng: Ứng dụng Vi điều khiển để thực hiện các tác vụ thông qua Board STM32

Đây là đồ án môn học **Hệ thống nhúng (HK2 - 2024-2025)** của Nhóm 9 - Lớp 22NHUNG, thực hiện tại Khoa Điện tử - Viễn thông, trường Đại học Khoa học Tự nhiên - ĐHQG TP.HCM.

Dự án này tập trung vào việc xây dựng một hệ thống nhúng có khả năng thực hiện nhiều tác vụ gần như đồng thời trên vi điều khiển STM32, sử dụng **Hệ điều hành thời gian thực (RTOS)** để quản lý và điều phối.

## 🎯 Chức năng chính

Hệ thống được thiết kế để thực hiện đồng thời các tác vụ sau:

*   **Quản lý đa nhiệm với FreeRTOS:** Xây dựng và quản lý các luồng (task) độc lập với các mức độ ưu tiên khác nhau, xử lý chuyển đổi trạng thái (suspend, resume) một cách linh hoạt.
*   **Giao diện LCD & Cảm ứng:**
    *   Hiển thị giao diện người dùng, bao gồm các "nút nhấn" ảo đại diện cho từng tác vụ.
    *   Nhận tín hiệu từ màn hình cảm ứng để kích hoạt (resume) các tác vụ con tương ứng.
*   **Giao tiếp CAN Bus (Task 02-2):**
    *   Thiết lập giao tiếp giữa 2 module CAN.
    *   CAN1 định kỳ (500ms) gửi một message chứa [Số nhóm] và [Nhiệt độ nội tại của chip].
    *   CAN2 lắng nghe và nhận message đó để hiển thị lên LCD, kiểm tra tính toàn vẹn của việc truyền nhận.
*   **Đọc dữ liệu từ Thẻ nhớ SD Card (Task 02-3):**
    *   Sử dụng hệ thống file FATFS để đọc nội dung từ một file `.txt` có sẵn trên thẻ nhớ.
    *   Hiển thị nội dung đọc được lên màn hình LCD.
*   **Lưu trữ dữ liệu vào bộ nhớ FRAM (Task 02-4):**
    *   Khi có sự kiện nhấn nút cứng, hệ thống đọc giá trị nhiệt độ nội tại của chip và thời gian hiện tại.
    *   Ghi dữ liệu này vào bộ nhớ không bay hơi FRAM qua giao thức I2C.
*   **Điều khiển LED chỉ thị (Task 02-1):**
    *   Nhấp nháy một đèn LED với chu kỳ 1 giây (0.5s sáng, 0.5s tắt) để minh họa một tác vụ tuần hoàn đơn giản.

## 🛠️ Công nghệ & Linh kiện

### Phần cứng
*   **Vi điều khiển:** STM32F405RGT6 (trên board Open405R-C)
*   **Màn hình:** TFT LCD 2.8 inch cảm ứng điện trở (IC hiển thị HX8347D & IC cảm ứng XPT2046)
*   **Giao tiếp:** 2x Module CAN Transceiver
*   **Lưu trữ:**
    *   Thẻ nhớ MicroSD Card
    *   Bộ nhớ FRAM FM24CL16B (16-Kbit)
*   **Các thành phần khác:** Nút nhấn, Đèn LED

### Phần mềm
*   **Hệ điều hành:** FreeRTOS (thông qua API CMSIS-RTOS v2)
*   **Ngôn ngữ:** C/C++
*   **Thư viện:** STM32 HAL, FATFS
*   **Môi trường phát triển:** STM32CubeIDE

## 📸 Hình ảnh & Video Demo

**Video demo hoạt động:**
[Xem video demo trên Youtube](https://youtube.com/shorts/6ENZYUoNgHU) 

## 👥 Thành viên nhóm

| STT | Họ và tên          | MSSV     |
|:---:|--------------------|:--------:|
| 1   | Hồng Quốc Bảo      | 22200011 |
| 2   | Trần Văn Điệp      | 22200031 |
| 3   | Nguyễn Hữu Duy     | 22200042 |
| 4   | Lê Bá Hiển         | 22200057 |
| 5   | Nguyễn Đại Minh Huy | 22200073 |

**Giảng viên hướng dẫn:** Thầy Hoàng Anh Tuấn
