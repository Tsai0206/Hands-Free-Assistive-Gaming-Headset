/*
 * MPU6050 Game Controller for Arduino Leonardo
 * 版本：2.0 (增強型序列輸出)
 * 描述：此版本修改了序列監視器的輸出，使其能清楚地顯示
 * 當前被按下的按鍵以及觸發該按鍵的感測器數值。
 *
 * 程式碼修改：程式夥伴
 * 現在時間：2025年10月11日 星期六 上午7:27:14
 * 地點：台灣
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>

// 建立 MPU6050 物件
Adafruit_MPU6050 mpu;

// --- 可調整參數 ---
// 傾斜觸發閾值 (單位：度)：控制 WASD。角度越大越不靈敏。
const int TILT_THRESHOLD = 20; 
// 旋轉觸發閾值 (單位：rad/s)：控制 Q/E。數值越大越不靈敏。
const float YAW_THRESHOLD = 1.5;

// --- 全域變數 ---
float gyroZ_offset = 0.0; // 僅為 Z 軸陀螺儀校準
char lastKeyPressed = 0;  // 追蹤上一個被按下的按鍵，0 代表沒有按鍵

void setup() {
  Serial.begin(115200); // 建議使用較高的鮑率以獲得更好的性能

  if (!mpu.begin()) {
    Serial.println("找不到 MPU6050 晶片");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 已找到!");
  
  // --- Z 軸陀螺儀校準 (僅用於 Q/E) ---
  Serial.println("正在校準陀螺儀 Z 軸... 請勿移動感測器。");
  delay(1000);
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZ_offset += g.gyro.z;
    delay(3);
  }
  gyroZ_offset /= 500;
  Serial.println("校準完成。控制器準備就緒。");

  // 初始化鍵盤功能
  Keyboard.begin();
}

void loop() {
  // 獲取感測器數據
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- 計算所需數值 ---
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  float calibrated_gz = g.gyro.z - gyroZ_offset;

  // --- 核心控制邏輯與狀態訊息產生 ---
  char desiredKey = 0;
  String statusMessage = "狀態: IDLE"; // 預設訊息為「閒置」

  // 優先級 1：檢查 Q/E (視角轉動)
  if (calibrated_gz > YAW_THRESHOLD) {
    desiredKey = 'e';
    statusMessage = "按鍵: E (向右轉) | 觸發數值: " + String(calibrated_gz);
  } else if (calibrated_gz < -YAW_THRESHOLD) {
    desiredKey = 'q';
    statusMessage = "按鍵: Q (向左轉) | 觸發數值: " + String(calibrated_gz);
  }

  // 優先級 2：如果沒有視角轉動，再檢查 WASD (移動)
  if (desiredKey == 0) {
    if (pitch > TILT_THRESHOLD) {
      desiredKey = 'w';
      statusMessage = "按鍵: W (前進)   | 觸發數值: " + String(pitch);
    } else if (pitch < -TILT_THRESHOLD) {
      desiredKey = 's';
      statusMessage = "按鍵: S (後退)   | 觸發數值: " + String(pitch);
    } else if (roll > TILT_THRESHOLD) {
      desiredKey = 'a';
      statusMessage = "按鍵: A (向左)   | 觸發數值: " + String(roll);
    } else if (roll < -TILT_THRESHOLD) {
      desiredKey = 'd';
      statusMessage = "按鍵: D (向右)   | 觸發數值: " + String(roll);
    }
  }

  // --- 按鍵狀態管理 ---
  if (desiredKey != lastKeyPressed) {
    if (lastKeyPressed != 0) {
      Keyboard.release(lastKeyPressed);
    }
    if (desiredKey != 0) {
      Keyboard.press(desiredKey);
    }
    lastKeyPressed = desiredKey;
  }
  
  // --- 印出最終的狀態訊息 ---
  Serial.println(statusMessage);

  delay(50); // 稍微加長延遲，讓訊息更容易閱讀
}