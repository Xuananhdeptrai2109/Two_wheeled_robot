# Hệ Thống Điều Khiển Xe Robot Tự Hành / Từ Xa Qua Wi-Fi (ESP32 + Python Flask)

Dự án này là hệ thống điều khiển xe robot 2 bánh vi sai sử dụng vi điều khiển **ESP32**, kết hợp trạm điều khiển máy tính/điện thoại qua **Python Flask Web Server** và giao thức truyền tin **UDP siêu tốc (độ trễ cực thấp)**.

Hệ thống tích hợp thuật toán **PID đồng tốc 2 bánh** bằng Encoder, công nghệ **đa luồng FreeRTOS (Dual-Core)**, cảm biến đo góc nghiêng/quán tính **MPU-6050** và cụm 2 cảm biến đo khoảng cách laser ToF **VL53L0X** để tự động phanh khẩn cấp chống va chạm.

---

## 📑 Mục Lục
1. [Kiến Trúc & Nguyên Lý Hoạt Động](#1-kiến-trúc--nguyên-lý-hoạt-động)
2. [Sơ Đồ Đấu Nối Mạch Phần Cứng (Wiring Diagram)](#2-sơ-đồ-đấu-nối-mạch-phần-cứng-wiring-diagram)
3. [Chuẩn Bị Phần Cứng & Thư Viện](#3-chuẩn-bị-phần-cứng--thư-viện)
4. [Hướng Dẫn Cài Đặt & Chạy Dự Án](#4-hướng-dẫn-cài-đặt--chạy-dự-án)
5. [Cơ Chế Điều Khiển & Tính Năng Nổi Bật](#5-cơ-chế-điều-khiển--tính-năng-nổi-bật)
6. [Xử Lý Lỗi Thường Gặp (Troubleshooting)](#6-xử-lý-lỗi-thường-gặp-troubleshooting)

---

## 1. Kiến Trúc & Nguyên Lý Hoạt Động

```text
  [ Trình duyệt Web (PC / Phone) ]
                 │
                 │ HTTP (POST /send_command)
                 ▼
     [ Python Flask Server (app.py) ]
                 │
                 │ Gói tin UDP socket (Port 12345)
                 ▼
   ┌──────────────────────────────────────────────────┐
   │                  ESP32 DEVKIT                    │
   │                                                  │
   │  [ Core 0: sensorTaskCore0 ]                     │
   │    ├── Đọc cảm biến Laser VL53L0X (Trái/Phải)    │
   │    └── Đọc góc quay GyroZ MPU-6050               │
   │                                                  │
   │  [ Core 1: loop() ]                              │
   │    ├── Nhận lệnh UDP tốc độ bánh (SpeedL,SpeedR) │
   │    ├── Tự động phanh nếu gặp vật cản < 20cm      │
   │    ├── Ngắt Encoder đếm xung bánh xe             │
   │    └── Thuật toán PID cân bằng đồng tốc 2 bánh   │
   │                                                  │
   │  [ Mạch Cầu H (L298N / TB6612FNG) ]             │
   │    ├── Động cơ DC Trái                           │
   │    └── Động cơ DC Phải                           │
   └──────────────────────────────────────────────────┘
```

---

## 2. Sơ Đồ Đấu Nối Mạch Phần Cứng (Wiring Diagram)

> [!IMPORTANT]
> **Quy tắc cấp nguồn:**
> - Tuyệt đối **KHÔNG** dùng nguồn 3.3V hoặc 5V của chân ESP32 để nuôi trực tiếp động cơ. Động cơ phải lấy nguồn riêng từ khối Pin (2 cell 18650 nối tiếp = 7.4V - 8.4V).
> - **BẮT BUỘC NỐI CHUNG GND (Mass)** giữa Nguồn Pin, Mạch Cầu H, ESP32 và các Cảm biến để các mức logic hoạt động chính xác.

### Bảng Chi Tiết Kết Nối Chân GPIO Trên ESP32

| Khối Chức Năng | Chân Trên Module / Cảm Biến | Chân GPIO Trên ESP32 | Ghi Chú Kỹ Thuật |
| :--- | :--- | :--- | :--- |
| **Động Cơ Trái** | **PWM_L** (ENA / PWM) | **GPIO 25** | Tín hiệu PWM điều chỉnh tốc độ (0 - 255) |
| | **DIR1_L** (IN1) | **GPIO 26** | Chân định chiều quay động cơ trái |
| | **DIR2_L** (IN2) | **GPIO 27** | Chân định chiều quay động cơ trái |
| **Động Cơ Phải** | **PWM_R** (ENB / PWM) | **GPIO 14** | Tín hiệu PWM điều chỉnh tốc độ (0 - 255) |
| | **DIR1_R** (IN3) | **GPIO 12** | Chân định chiều quay động cơ phải |
| | **DIR2_R** (IN4) | **GPIO 13** | Chân định chiều quay động cơ phải |
| **Encoder Trái** | Kênh A (Signal A) | **GPIO 34** | Ngắt ngoài đọc xung (Input Only, cần trở treo ngoài nếu encoder hở cực) |
| | Kênh B (Signal B) | **GPIO 35** | Chiều quay encoder (Input Only) |
| | VCC / GND | 3.3V hoặc 5V / GND | Nguồn nuôi mắt đọc encoder |
| **Encoder Phải** | Kênh A (Signal A) | **GPIO 32** | Ngắt ngoài đọc xung (Cấu hình INPUT_PULLUP) |
| | Kênh B (Signal B) | **GPIO 33** | Chiều quay encoder (Cấu hình INPUT_PULLUP) |
| | VCC / GND | 3.3V hoặc 5V / GND | Nguồn nuôi mắt đọc encoder |
| **Bus I2C (Chung)** | **SDA** (Chung MPU + 2x VL53) | **GPIO 21** | Đường truyền dữ liệu I2C |
| | **SCL** (Chung MPU + 2x VL53) | **GPIO 22** | Đường xung nhịp I2C |
| **Cảm biến MPU-6050** | VCC / GND | 3.3V (hoặc 5V) / GND | Nguồn cảm biến |
| | AD0 | Nối GND | Thiết lập địa chỉ I2C mặc định là `0x68` |
| **Laser ToF 1 (J3 - Trái)** | **XSHUT** | **GPIO 19** | Reset & nạp lại địa chỉ I2C thành `0x30` |
| | VCC / GND | 3.3V (hoặc 5V) / GND | Nguồn cảm biến |
| **Laser ToF 2 (J4 - Phải)** | **XSHUT** | **GPIO 18** | Reset & nạp lại địa chỉ I2C thành `0x31` |
| | VCC / GND | 3.3V (hoặc 5V) / GND | Nguồn cảm biến |

### Sơ Đồ Cấp Nguồn Khuyến Nghị

```text
[ Khối Pin 7.4V - 8.4V (2S 18650) ]
        ├── (+) ──────> VCC/Power In Cầu H (L298N / TB6612)
        │
        ├── (Buck hạ áp 5V) ──> Chân Vin (hoặc 5V) của Mạch ESP32
        │
        └── (-) GND ──┬───────> GND Cầu H
                      ├───────> GND ESP32
                      ├───────> GND Cảm biến MPU6050 & VL53L0X
                      └───────> GND Encoders
                      (Tất cả GND phải thông với nhau)
```

---

## 3. Chuẩn Bị Phần Cứng & Thư Viện

### 3.1. Danh Sách Linh Kiện
1. 01 x Mạch điều khiển **ESP32 DevKit V1** (30 chân hoặc 38 chân).
2. 01 x Mạch điều khiển động cơ cầu H (khuyến nghị **TB6612FNG** để tiết kiệm nhiệt và nhỏ gọn, hoặc **L298N**).
3. 02 x Động cơ DC giảm tốc kèm đĩa Encoder (loại encoder quang hoặc hall).
4. 02 x Module cảm biến khoảng cách Laser ToF **VL53L0X**.
5. 01 x Module cảm biến gia tốc / con quay 6 trục **MPU-6050**.
6. 01 x Khung gầm robot 2 bánh vi sai + bánh xe dẫn hướng (bánh mắt trâu caster wheel).
7. Nguồn: 02 pin 18650 kèm đế pin, mạch hạ áp 5V LM2596/Buck mini (để cấp cho ESP32).

### 3.2. Cài Đặt Thư Viện Trên Arduino IDE
Mở **Arduino IDE** -> Vào **Tools** -> **Manage Libraries...** (hoặc nhấn `Ctrl + Shift + I`), tìm kiếm và cài đặt:
1. `Adafruit VL53L0X` (by Adafruit)
2. `Adafruit MPU6050` (by Adafruit)
3. `Adafruit Unified Sensor` (by Adafruit)
4. Board package: Vào **Preferences** thêm URL ESP32 board và cài đặt gói board **esp32** trong **Boards Manager**.

### 3.3. Cài Đặt Phía Máy Tính (Laptop)
Yêu cầu cài đặt **Python 3.8+**. Mở Terminal / Command Prompt / PowerShell và chạy:
```bash
pip install Flask
```

---

## 4. Hướng Dẫn Cài Đặt & Chạy Dự Án

### Bước 1: Nạp Code Lên Xe (ESP32)
1. Mở file [sketch_sep16a/sketch_sep16a.ino](file:///d:/NVT%20-%20IOT/test_python/sketch_sep16a/sketch_sep16a.ino) bằng phần mềm **Arduino IDE**.
2. Tìm đến dòng 9 và dòng 10:
   ```cpp
   const char* ssid = "TEN_WIFI_CUA_BAN";      // Đổi tên Wi-Fi
   const char* password = "MAT_KHAU_WIFI";     // Đổi mật khẩu
   ```
   *(Lưu ý: Nên dùng mạng Wi-Fi 2.4GHz từ Router hoặc phát từ điện thoại 4G).*
3. Cắm cáp Micro-USB/Type-C nối ESP32 với máy tính.
4. Chọn đúng Board: **ESP32 Dev Module** và cổng **COM** tương ứng.
5. Nhấn **Upload** (Mũi tên sang phải).
6. Nạp xong, mở **Serial Monitor** (góc trên bên phải), chỉnh tốc độ truyền (Baudrate) thành `115200`.
7. Nhấn nút cứng **EN / RST** trên mạch ESP32. Chờ màn hình in thông báo:
   ```text
   Kết nối Wi-Fi THÀNH CÔNG!
   => ĐỊA CHỈ IP CỦA XE LÀ: 192.168.x.x
   ```
   👉 **Sao chép lại địa chỉ IP này** (Ví dụ: `192.168.1.15`).

### Bước 2: Cập Nhật IP Xe Vào Server Python
1. Mở file [app.py](file:///d:/NVT%20-%20IOT/test_python/app.py).
2. Tìm đến dòng số 8:
   ```python
   ESP32_IP = "192.168.x.x"  # Dán IP vừa lấy được từ bước 1 vào đây
   ```
3. Lưu file lại (`Ctrl + S`).
4. Khởi chạy Server Flask bằng lệnh:
   ```bash
   python app.py
   ```
   Màn hình sẽ hiển thị Server đang lắng nghe tại: `http://0.0.0.0:5000` (hoặc `http://127.0.0.1:5000`).

### Bước 3: Điều Khiển Xe
1. Mở trình duyệt Web (Chrome, Edge, Cốc Cốc, Safari...).
2. Truy cập: `http://127.0.0.1:5000` (nếu dùng điện thoại cùng mạng Wi-Fi thì gõ `http://<IP_MAY_TINH>:5000`).
3. Giao diện điều khiển hiển thị sẵn sàng.

---

## 5. Cơ Chế Điều Khiển & Tính Năng Nổi Bật

### 5.1. Thao Tác Điều Khiển
* **Chuột & Màn hình cảm ứng**:
  * **Bấm và GIỮ** nút (Tiến, Lùi, Trái, Phải) để xe di chuyển.
  * **NHẢ TAY** ra xe sẽ tự động kích hoạt phanh dừng lại ngay tức thì.
  * Nút đỏ **Dừng** ở giữa để phanh khẩn cấp.
* **Bàn phím máy tính**:
  * Phím tiến: <kbd>W</kbd> hoặc <kbd>↑</kbd>
  * Phím lùi: <kbd>S</kbd> hoặc <kbd>↓</kbd>
  * Phím rẽ trái: <kbd>A</kbd> hoặc <kbd>←</kbd>
  * Phím rẽ phải: <kbd>D</kbd> hoặc <kbd>→</kbd>
  * Phanh gấp: Phím <kbd>Space</kbd> (phím cách).

### 5.2. Các Tính Năng Kỹ Thuật Độc Đáo
1. **Phanh Điện Từ Hãm Cứng (Short Brake)**: Khác với việc ngắt điện thông thường khiến xe trôi theo quán tính, hàm `driveMotorLeft(0, true)` kích cả 2 chân cực động cơ lên mức logic `HIGH` và cấp xung PWM 255. Động cơ sẽ bị hãm khóa tức thì, tăng độ an toàn.
2. **Tự Động Phanh Trước Vật Cản (Radar ToF Laser)**: Nếu xe đang chạy tiến mà 1 trong 2 cảm biến khoảng cách phát hiện vật cản gần hơn **200 mm (20 cm)**, xe sẽ tự hủy lệnh chạy và khóa bánh phanh khẩn cấp.
3. **Cân Bằng Đồng Tốc Bánh Xe Bằng PID**: Bộ điều chỉnh tích phân sai số xung encoder (`Kp_sync = 1.8`, `Ki_sync = 0.25`) liên tục bù trừ độ lệch lực giữa 2 bánh mỗi **30 ms**, giúp xe đi thẳng tắp mà không bị lệch lái.
4. **FreeRTOS Dual-Core Chống Nghẽn**: Tác vụ đo laser (ToF) tốn 20-30ms mỗi chu kỳ được tách riêng sang **Core 0**, giải phóng hoàn toàn **Core 1** để nhận gói UDP và đáp ứng lệnh điều khiển với độ trễ < 5ms.

---

## 6. Xử Lý Lỗi Thường Gặp (Troubleshooting)

| Hiện Tượng | Nguyên Nhân | Cách Khắc Phục |
| :--- | :--- | :--- |
| **Serial báo "KHÔNG TÌM THẤY THIẾT BỊ I2C"** | Lỏng dây SDA/SCL hoặc chưa nối chung GND | Kiểm tra lại GPIO 21 (SDA), GPIO 22 (SCL). Đo kiểm tra dây nối mass giữa các module. |
| **Xe không nhận lệnh khi bấm nút** | Sai IP ESP32 trong `app.py` hoặc khác lớp mạng | Đảm bảo Laptop và ESP32 cùng kết nối 1 mạng Wi-Fi (tránh mạng Wi-Fi công cộng có cơ chế chặn giao tiếp nội bộ AP Isolation). Kiểm tra kỹ IP trong `app.py`. |
| **Một bên bánh xe quay ngược chiều** | Đấu dây động cơ ngược cực | Đổi chéo 2 dây ngõ ra motor tại cầu H (OUT1/OUT2 hoặc OUT3/OUT4) hoặc hoán đổi logic chân `dir1`/`dir2` trong code. |
| **Xe tự giật phanh liên tục dù không có vật cản** | Cảm biến VL53L0X bị bẩn, dính vật phản xạ quá gần hoặc cấu hình ngưỡng cản quá nhạy | Kiểm tra thông số in ra Serial Monitor xem cảm biến nào đang đọc < 200mm. Vệ sinh mắt phát/thu laser. |
| **Động cơ kêu "è è" nhưng không quay** | Pin yếu không đủ dòng hoặc hệ số PWM quá thấp | Sạc đầy 2 pin 18650. Đảm bảo nguồn cấp cho cầu H từ 7.4V trở lên. |