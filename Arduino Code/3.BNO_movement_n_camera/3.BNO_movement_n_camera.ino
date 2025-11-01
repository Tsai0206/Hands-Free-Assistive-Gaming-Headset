/*
 * BNO055 Movement and Camera View Test
 * Version: 9.9.1 (Typo Fix)
 * Description: 
 * 1. Waits for FULL BNO055 sensor calibration (SYS=3) in setup().
 * 2. Provides in-game "twitch" feedback if calibration is lost.
 * 3. (FIX) Corrected 'AdaA_BNO055' and 'clutchIsEngSaged' typos.
 *
 * Code Modified by: Code Companion
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Keyboard.h>
#include <Mouse.h>

// 使用 I2C address 0x29
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// --- Tweakable Parameters ---
// -- BNO055 Movement Control (based on Yaw - Euler X) --
const int YAW_WALK_THRESHOLD = 15;    // 轉左 (Yaw 負值) 多少度開始走路
const int YAW_SPRINT_THRESHOLD = 20; // 轉左 (Yaw 負值) 多少度開始衝刺
const int YAW_BACKUP_THRESHOLD = 70; // 轉右 (Yaw 正值) 多少度開始後退

// -- BNO055 Mouse Clutch Control (FlexiForce Sensor) --
const int FLEX_PIN = A1;
int FLEX_CLUTCH_THRESHOLD = 700;
float mouseMultiplierX = 1.5; // << 已根據你的抖動問題調低
float GYRO_DEADZONE = 0.2;  // << 已根據你的抖動問題調高
const int MAX_MOUSE_SPEED_PER_FRAME = 25;

// --- State Tracking ---
struct KeyStates {
  bool w = false, s = false, left_ctrl = false;
};
KeyStates keyStates;

float neutralYawOffset = 0.0;

// --- Helper Functions ---
void handleKeyPress(uint8_t key, bool shouldBePressed, bool &isPressed) {
  if (shouldBePressed && !isPressed) {
    Keyboard.press(key); isPressed = true;
  } else if (!shouldBePressed && isPressed) {
    Keyboard.release(key); isPressed = false;
  }
}

void setup() {
  Serial.begin(115200); 

  // --- 1. BNO055 初始化 ---
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected...");
    while (1);
  }
  Serial.println("BNO055 Found!");
  bno.setExtCrystalUse(true);


  // --- 2. (新功能) 等待感測器完整校準 ---
  Serial.println("--- 完整感測器校準 ---");
  Serial.println("請拿起感測器在空中畫 8 字型");
  Serial.println("直到所有狀態都變為 3。");
  

  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  while (sys != 3) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    
    Serial.print("校準狀態: ");
    Serial.print("系統(SYS):"); Serial.print(sys, DEC);
    Serial.print(" 陀螺(G):"); Serial.print(gyro, DEC);
    Serial.print(" 加速(A):"); Serial.print(accel, DEC);
    Serial.print(" 磁力(M):"); Serial.println(mag, DEC);
    delay(100); 
  }
  Serial.println("\n--- 完整校準完成! ---");


  // --- 3. 初始化鍵盤與滑鼠 ---
  Keyboard.begin();
  Mouse.begin();


  // --- 4. 校準「中立 Yaw 值」 ---
  Serial.println("校準「中立 Yaw 值」... 請將感測器朝向前方。");
  delay(2000); 

  sensors_event_t orientationData;
  float yawSum = 0;
  int calibrationSamples = 50;
  for (int i = 0; i < calibrationSamples; i++) {
     bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
     yawSum += orientationData.orientation.x;
     delay(10);
  }
  neutralYawOffset = yawSum / calibrationSamples;
  while (neutralYawOffset >= 360) neutralYawOffset -= 360;
  while (neutralYawOffset < 0) neutralYawOffset += 360;

  Serial.print("中立 Yaw Offset 計算完成: ");
  Serial.println(neutralYawOffset);
  Serial.println("BNO055 Movement/View Test 準備完成。");
}

// (舊功能) 計算最短角度差的函數
float calculateAngleDifference(float currentAngle, float targetAngle) {
  float diff = currentAngle - targetAngle;
  while (diff <= -180) diff += 360;
  while (diff > 180) diff -= 360;
  return diff;
}

void loop() {
  // --- 在迴圈開始時檢查校準狀態 ---
  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // --- 主要邏輯只在「已校準」時執行 ---
  if (sys == 3) {
    
    // --- 這是你原本的 loop() 程式碼 ---
    sensors_event_t orientationData;
    
    // *** 修正 #1 ***
    // 之前打成 'AdaA_BNO055'
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    // --- BNO055 鍵盤移動控制 (Yaw-based) ---
    float rawYaw = orientationData.orientation.x; 
    float relativeYaw = calculateAngleDifference(rawYaw, neutralYawOffset);

    bool should_w_be_pressed = false;
    bool should_s_be_pressed = false;
    bool should_ctrl_be_pressed = false;
    String moveState = "IDLE";

    // --- Inverted Yaw Mapping ---
    if (relativeYaw < -YAW_SPRINT_THRESHOLD) {
      moveState = "SPRINTING";
      should_w_be_pressed = true; should_ctrl_be_pressed = true;
    } else if (relativeYaw < -YAW_WALK_THRESHOLD) {
      moveState = "WALKING";
      should_w_be_pressed = true;
    }
    else if (relativeYaw > YAW_BACKUP_THRESHOLD) {
      moveState = "BACKING UP";
      should_s_be_pressed = true;
    }

    handleKeyPress('w', should_w_be_pressed, keyStates.w);
    handleKeyPress('s', should_s_be_pressed, keyStates.s);
    handleKeyPress(KEY_LEFT_CTRL, should_ctrl_be_pressed, keyStates.left_ctrl);

    // --- 讀取 FlexiForce Sensor (Clutch Only) ---
    int flexValue = analogRead(FLEX_PIN);
    bool clutchIsEngaged = (flexValue > FLEX_CLUTCH_THRESHOLD);

    // --- 離合器啟動的滑鼠相機控制 (using BNO055 Gyro X) ---
    
    // *** 修正 #2 ***
    // 之前打成 'clutchIsEngSaged'
    if (clutchIsEngaged) {
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      float gyro_x_rad_s = gyro.x(); 
      int mouse_vx = 0;
      if (abs(gyro_x_rad_s) > GYRO_DEADZONE) {
        mouse_vx = (int)(gyro_x_rad_s * mouseMultiplierX);
        mouse_vx = constrain(mouse_vx, -MAX_MOUSE_SPEED_PER_FRAME, MAX_MOUSE_SPEED_PER_FRAME);
      }
      if (mouse_vx != 0) {
        Mouse.move(mouse_vx, 0);
      }
    }

    // --- Serial Monitor Output ---
    Serial.print("Move: " + moveState + " (Rel Yaw: " + String(relativeYaw, 1) + ")");
    Serial.print(" | Mouse Clutch: ");
    if (clutchIsEngaged) { Serial.print("ON"); } else { Serial.print("OFF"); }
    Serial.println(" | Flex: " + String(flexValue));

    delay(10); 

  } 
  else {
    // --- (新功能) 安全停止：如果校準遺失 ---
    Serial.println("!!! 校準遺失 !!! 正在停止所有動作...");

    // 1. 立刻釋放所有按鍵 (安全第一)
    handleKeyPress('w', false, keyStates.w);
    handleKeyPress('s', false, keyStates.s);
    handleKeyPress(KEY_LEFT_CTRL, false, keyStates.left_ctrl);

    // 2. (新功能) 遊戲內反饋：
    // 快速按下並釋放 'w' 鍵，讓角色「頓」一下
    // 讓你知道是時候重新校準了
    Keyboard.press('w');
    delay(50); // 按下 50 毫秒
    Keyboard.release('w');
    
    delay(100); // 稍作等待再重新檢查
  }
}