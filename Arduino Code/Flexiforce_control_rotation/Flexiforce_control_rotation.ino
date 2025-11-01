/*
 * 最終整合版：MPU6050 連續滑鼠控制 + FlexiForce 壓力離合器
 * 描述：只有當 FlexiForce 壓力感測器的壓力超過閾值時，
 * 才會啟動陀螺儀的讀取和滑鼠移動功能。
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Mouse.h>

// --- 新增硬體腳位定義 ---
const int flexPin = A1; // FlexiForce 感測器連接的類比腳位

// --- 可調整參數 ---
// 新增壓力觸發閾值
int flexThreshold = 850; 
float mouseMultiplierX = 45.0; 
float mouseMultiplierZ = 20.0; 
float deadzoneThreshold = 0.1;

// --- 全域變數 ---
Adafruit_MPU6050 mpu;
float gyroX_offset = 0.0;
float gyroZ_offset = 0.0;

void setup() {
  Serial.begin(115200); // 建議使用較高的鮑率以獲得更好的性能
  
  if (!mpu.begin()) {
    Serial.println("找不到 MPU6050 晶片");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 已找到!");

  // --- 陀螺儀自動校準 ---
  Serial.println("正在校準陀螺儀... 請勿移動感測器。");
  delay(1000);
  const int calibrationSamples = 500;
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroX_offset += g.gyro.x;
    gyroZ_offset += g.gyro.z;
    delay(3);
  }
  gyroX_offset /= calibrationSamples;
  gyroZ_offset /= calibrationSamples;

  Serial.println("校準完成。");
  // --- 新增提示訊息 ---
  Serial.println("FlexiForce Clutch system is ready. Press sensor to activate tracking.");
  
  Mouse.begin();
}

void loop() {
  // --- 核心修改點：所有操作都基於這個 if 判斷 ---

  // 1. 讀取壓力感測器的值
  int flexValue = analogRead(flexPin);

  // 2. 檢查壓力是否超過閾值 (離合器是否被踩下)
  if (flexValue > flexThreshold) { // <--- 這是新增的「總開關」
    
    // --- 以下是原始程式碼 loop() 的全部內容 ---
    
    // 取得感測器新數據
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 套用校準偏移量
    float calibrated_gx = g.gyro.x - gyroX_offset;
    float calibrated_gz = g.gyro.z - gyroZ_offset;

    int mouse_vx = 0;
    int mouse_vy = 0;

    // 將 Z 軸旋轉 (gz) 映射到滑鼠的 X 軸移動 (左右)
    if (abs(calibrated_gz) > deadzoneThreshold) {
      mouse_vx = -(int)(calibrated_gz * mouseMultiplierX);
    }
    // 將 X 軸旋轉 (gx) 映射到滑鼠的 Y 軸移動 (上下)
    if (abs(calibrated_gx) > deadzoneThreshold) {
      mouse_vy = (int)(calibrated_gx * mouseMultiplierZ);
    }

    // 移動滑鼠
    if (mouse_vx != 0 || mouse_vy != 0) {
      Mouse.move(mouse_vx, mouse_vy);
    }

    // --- 修改了序列輸出，使其包含模式狀態 ---
    Serial.print("Rotation Mode: ON   | Flex = ");
    Serial.print(flexValue);
    Serial.print(" | X = ");
    Serial.print(mouse_vx);
    Serial.print(" | Y = ");
    Serial.println(mouse_vy);

  } else {
    // 如果壓力不夠，離合器就是「鬆開」的狀態，不執行任何動作。
    Serial.print("Rotation Mode: OFF  | Flex = ");
    Serial.println(flexValue);
  }
  
  delay(20);
}