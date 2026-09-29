#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================= CẤU HÌNH WI-FI & MẠNG =================
const char* ssid = "TEN_WITF";   // TODO: Sửa tên Wi-Fi
const char* password = "MAT_KHAU_WIFI";  // TODO: Sửa mật khẩu
const int udpPort = 12345;               

WiFiUDP udp;
char packetBuffer[255]; 

// ================= KHỞI TẠO CHÂN (GPIO) =================
const int encALeft = 34, encBLeft = 35;
const int encARight = 32, encBRight = 33;

const int pwmLeft = 25, dir1Left = 26, dir2Left = 27; 
const int pwmRight = 14, dir1Right = 12, dir2Right = 13;

// ================= CẤU HÌNH CẢM BIẾN (I2C) =================
#define I2C_SDA 21 // Chuẩn SDA = GPIO 21
#define I2C_SCL 22 // Chuẩn SCL = GPIO 22
#define XSHUT_J3 19 // Chân bật/tắt cảm biến VL53L0X 1
#define XSHUT_J4 18 // Chân bật/tắt cảm biến VL53L0X 2

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

// Biến lưu trữ dữ liệu cảm biến (chia sẻ giữa Core 0 và Core 1)
volatile float currentGyroZ = 0.0;
volatile uint16_t distLeft = 0;
volatile uint16_t distRight = 0;
float gyroZ_offset = 0.0; // Bù sai số tĩnh MPU6050

// ================= BIẾN ĐẾM XUNG ENCODER =================
volatile long ticksLeft = 0;
volatile long ticksRight = 0;

void IRAM_ATTR readEncoderLeft() {
  ticksLeft++;
}

void IRAM_ATTR readEncoderRight() {
  ticksRight++;
}

// ================= THAM SỐ ĐIỀU KHIỂN & PID ĐỒNG TỐC =================
float targetSpeedL = 0.0;
float targetSpeedR = 0.0;

// NGƯỠNG TỰ ĐỘNG PHANH KHẨN CẤP (200 mm = 20 cm)
const uint16_t BRAKE_DISTANCE_MM = 200;

// HỆ SỐ BÙ TRỪ CƠ KHÍ
float RATIO_LEFT_MOTOR  = 1.00; 
float RATIO_RIGHT_MOTOR = 1.20; 

unsigned long prevMillis = 0;
const int interval = 30; // Chu kỳ cân bằng tốc độ (30 ms)

float syncIntegral = 0.0;
const float Kp_sync = 1.8;   
const float Ki_sync = 0.25;  

// ================= HÀM XUẤT LỰC & PHANH ĐỘNG CƠ =================
void driveMotorLeft(int pwmVal, bool forward) {
  if (pwmVal <= 0) {
    // PHANH GẤP ĐIỆN TỪ (Short Brake) - khóa cứng động cơ lập tức, chống trôi quán tính
    digitalWrite(dir1Left, HIGH);
    digitalWrite(dir2Left, HIGH);
    analogWrite(pwmLeft, 255);
  } else {
    if (forward) {
      digitalWrite(dir1Left, HIGH);
      digitalWrite(dir2Left, LOW);
    } else {
      digitalWrite(dir1Left, LOW);
      digitalWrite(dir2Left, HIGH);
    }
    analogWrite(pwmLeft, constrain(pwmVal, 0, 255));
  }
}

void driveMotorRight(int pwmVal, bool forward) {
  if (pwmVal <= 0) {
    // PHANH GẤP ĐIỆN TỪ (Short Brake) - khóa cứng động cơ lập tức, chống trôi quán tính
    digitalWrite(dir1Right, HIGH);
    digitalWrite(dir2Right, HIGH);
    analogWrite(pwmRight, 255);
  } else {
    if (forward) {
      digitalWrite(dir1Right, LOW);
      digitalWrite(dir2Right, HIGH);
    } else {
      digitalWrite(dir1Right, HIGH);
      digitalWrite(dir2Right, LOW);
    }
    analogWrite(pwmRight, constrain(pwmVal, 0, 255));
  }
}

void stopAllMotors() {
  driveMotorLeft(0, true);
  driveMotorRight(0, true);
  syncIntegral = 0;
}

// ================= HÀM KHỞI TẠO CẢM BIẾN =================
void setupSensors() {
  // 1. Kiểm tra trạng thái điện áp phần cứng đường dây I2C
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(10);
  int sdaState = digitalRead(I2C_SDA);
  int sclState = digitalRead(I2C_SCL);

  Serial.println("\n================ KIỂM TRA PHẦN CỨNG I2C ================");
  Serial.printf("Điện áp chân SDA (GPIO %d): %s\n", I2C_SDA, sdaState ? "HIGH (TỐT)" : "LOW (❌ BỊ CHẬP MASS / KẸT ĐƯỜNG DÂY!)");
  Serial.printf("Điện áp chân SCL (GPIO %d): %s\n", I2C_SCL, sclState ? "HIGH (TỐT)" : "LOW (❌ BỊ CHẬP MASS / KẸT ĐƯỜNG DÂY!)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100kHz

  pinMode(XSHUT_J3, OUTPUT);
  pinMode(XSHUT_J4, OUTPUT);

  // Thử kiểm tra riêng địa chỉ 0x68 của MPU
  Wire.beginTransmission(0x68);
  byte mpuErr = Wire.endTransmission();
  Serial.printf("Kiểm tra MPU-6050 (0x68): Mã phản hồi = %d (0: Thấy, 2: Không thấy, 4: Chập bus)\n", mpuErr);

  int nDevices = 0;
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("=> Tìm thấy thiết bị tại địa chỉ I2C: 0x%02X\n", address);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("❌ KHÔNG TÌM THẤY BẤT KỲ THIẾT BỊ I2C NÀO!");
  }
  Serial.println("========================================================\n");

  // ================= KHỞI TẠO MPU-6050 =================
  // Đọc mã nhận diện chip (WHO_AM_I - thanh ghi 0x75)
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  byte chipId = Wire.read();
  Serial.printf("Mã Chip ID của MPU (WHO_AM_I): 0x%02X (Chuẩn MPU6050 là 0x68, MPU6500 là 0x70)\n", chipId);

  // Đánh thức MPU khỏi chế độ ngủ (Sleep Mode)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Thanh ghi quản lý nguồn PWR_MGMT_1
  Wire.write(0x00); // Ghi 0 để đánh thức chip
  Wire.endTransmission();

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Lưu ý: Thư viện Adafruit_MPU6050 kiểm tra mã chip chặt chẽ.");
  } else {
    Serial.println(" Khởi tạo MPU-6050 THÀNH CÔNG!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Hiệu chuẩn độ lệch GyroZ khi xe đứng yên
    Serial.print("Đang hiệu chuẩn GyroZ MPU6050...");
    float sumZ = 0.0;
    const int calibSamples = 80;
    for (int i = 0; i < calibSamples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      sumZ += g.gyro.z;
      delay(5);
    }
    gyroZ_offset = sumZ / calibSamples;
    Serial.printf(" Xong! Offset: %.4f rad/s\n", gyroZ_offset);
  }

  // ================= KHỞI TẠO VL53L0X =================
  // Reset 2 cảm biến VL53L0X
  digitalWrite(XSHUT_J3, LOW);
  digitalWrite(XSHUT_J4, LOW);
  delay(20);

  // Khởi tạo J3 (Đổi sang 0x30)
  digitalWrite(XSHUT_J3, HIGH);
  delay(20);
  if (!lox1.begin(LOX1_ADDRESS, false, &Wire)) {
    Serial.println("Lỗi: Không tìm thấy VL53L0X (J3)");
  }

  // Khởi tạo J4 (Đổi sang 0x31)
  digitalWrite(XSHUT_J4, HIGH);
  delay(20);
  if (!lox2.begin(LOX2_ADDRESS, false, &Wire)) {
    Serial.println("Lỗi: Không tìm thấy VL53L0X (J4)");
  }
}

// ================= TASK ĐỌC CẢM BIẾN NGẦM (CHẠY TRÊN CORE 0) =================
void sensorTaskCore0(void *pvParameters) {
  for (;;) {
    // 1. Đọc dữ liệu MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    currentGyroZ = g.gyro.z - gyroZ_offset;

    // 2. Đọc khoảng cách VL53L0X (chờ đo ~30ms mỗi cảm biến trên Core 0, không ảnh hưởng Core 1)
    VL53L0X_RangingMeasurementData_t measure1;
    VL53L0X_RangingMeasurementData_t measure2;
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);

    distLeft = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : 9999;
    distRight = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : 9999;

    // Nghỉ 20ms trước lượt đo kế tiếp
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  
  pinMode(pwmLeft, OUTPUT); pinMode(dir1Left, OUTPUT); pinMode(dir2Left, OUTPUT);
  pinMode(pwmRight, OUTPUT); pinMode(dir1Right, OUTPUT); pinMode(dir2Right, OUTPUT);

  pinMode(encALeft, INPUT); pinMode(encBLeft, INPUT);
  pinMode(encARight, INPUT_PULLUP); pinMode(encBRight, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encALeft), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encARight), readEncoderRight, RISING);

  stopAllMotors();
  
  // Khởi tạo cụm cảm biến I2C
  setupSensors();

  // Tạo luồng đọc cảm biến độc lập chạy trên CORE 0 (Core 1 dành riêng cho PID & UDP)
  xTaskCreatePinnedToCore(
    sensorTaskCore0,   // Hàm thực thi task
    "SensorTask",      // Tên task
    4096,              // Kích thước Stack (byte)
    NULL,              // Tham số truyền vào
    1,                 // Độ ưu tiên task (1 là mức bình thường)
    NULL,              // Con trỏ quản lý task
    0                  // Chạy trên CORE 0
  );

  Serial.println("\nĐang kết nối Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  
  Serial.println("\nKết nối Wi-Fi THÀNH CÔNG!");
  Serial.print("=> ĐỊA CHỈ IP CỦA XE LÀ: ");
  Serial.println(WiFi.localIP());

  udp.begin(udpPort);
  Serial.printf("Đang lắng nghe UDP tại cổng %d\n", udpPort);
}

// ================= VÒNG LẶP CHÍNH (CHẠY TRÊN CORE 1) =================
void loop() {
  unsigned long currentMillis = millis();

  // 1. NHẬN LỆNH QUA UDP TỪ PYTHON
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 254);
    if (len > 0) packetBuffer[len] = '\0'; 
    
    float speed_L = 0.0, speed_R = 0.0;
    if (sscanf(packetBuffer, "%f,%f", &speed_L, &speed_R) == 2) {
      targetSpeedL = speed_L;
      targetSpeedR = speed_R;
      
      if (abs(targetSpeedL) < 0.05 && abs(targetSpeedR) < 0.05) {
        stopAllMotors();
      }
    }
  }

  // 2. VÒNG LẶP PID CÂN BẰNG TỐC ĐỘ 2 BÁNH THEO ENCODER (CHU KỲ CHUẨN XÁC 30ms)
  if (currentMillis - prevMillis >= interval) {
    prevMillis = currentMillis;

    // TỰ ĐỘNG PHANH KHẨN CẤP KHI GẶP VẬT CẢN PHÍA TRƯỚC (< 200mm)
    bool obstacleAhead = (distLeft < BRAKE_DISTANCE_MM || distRight < BRAKE_DISTANCE_MM);
    if (obstacleAhead && (targetSpeedL > 0 || targetSpeedR > 0)) {
      stopAllMotors(); // Phanh khẩn cấp giữ xe đứng im
      noInterrupts();
      ticksLeft = 0;
      ticksRight = 0;
      interrupts();

      static unsigned long lastWarn = 0;
      if (millis() - lastWarn >= 500) {
        lastWarn = millis();
        Serial.printf("⚠️ TỰ ĐỘNG PHANH! Phát hiện vật cản: [L: %d mm, R: %d mm]\n", distLeft, distRight);
      }
      return; // Tạm thời dừng xe, chờ đường thoáng
    }

    if (abs(targetSpeedL) < 0.05 && abs(targetSpeedR) < 0.05) {
      stopAllMotors();
      noInterrupts();
      ticksLeft = 0;
      ticksRight = 0;
      interrupts();
      return;
    }

    noInterrupts();
    long currentL = ticksLeft; ticksLeft = 0;
    long currentR = ticksRight; ticksRight = 0;
    interrupts();

    int basePWM_L = constrain((int)(abs(targetSpeedL) * 255.0 * RATIO_LEFT_MOTOR), 0, 255);
    int basePWM_R = constrain((int)(abs(targetSpeedR) * 255.0 * RATIO_RIGHT_MOTOR), 0, 255);

    int pwmL = basePWM_L;
    int pwmR = basePWM_R;
    long diff = 0;

    bool isGoingStraight = (targetSpeedL * targetSpeedR > 0);

    if (isGoingStraight) {
      float effectiveL = (float)currentL / RATIO_LEFT_MOTOR;
      float effectiveR = (float)currentR / RATIO_RIGHT_MOTOR;

      diff = (long)(effectiveL - effectiveR);
      syncIntegral += diff;
      syncIntegral = constrain(syncIntegral, -200.0, 200.0);

      int correction = (int)(Kp_sync * diff + Ki_sync * syncIntegral);
      correction = constrain(correction, -60, 60);

      pwmL = constrain(basePWM_L - correction, 40, 255);
      pwmR = constrain(basePWM_R + correction, 40, 255);
    } else {
      syncIntegral = 0;
    }

    driveMotorLeft(pwmL, targetSpeedL > 0);
    driveMotorRight(pwmR, targetSpeedR > 0);

    // In thông số ra Serial Monitor mỗi 150ms để kiểm tra (cả Encoder và Cảm biến)
    static int logCount = 0;
    if (++logCount >= 5) {
      logCount = 0;
      Serial.printf("Encoder [L: %2ld, R: %2ld] | Cam bien [L: %d mm, R: %d mm, GyroZ: %.2f]\n", 
                    currentL, currentR, distLeft, distRight, currentGyroZ);
    }
  }
}