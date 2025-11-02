/*
 * MPU6050 + Sip & Puff 完整控制器
 * Version: 10.0 (Final Integration)
 * Description: 
 * 1. 整合了 MPU6050 的雙模式移動/瞄準系統 (V9.1)。
 * 2. 整合了 Sip & Puff 的動作控制系統 (V2.5)。
 * 3. 兩個系統在 A0 和 A1 上獨立運作，功能完整合併。
 *
 * Code Integrated by: Code Companion
 */

// --- 共同函式庫 ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

// --- 感測器物件 ---
Adafruit_MPU6050 mpu;

// --- Tweakable Parameters (All parameters from both codes) ---

// -- MPU6050 Movement Control (Pitch-based) --
const int PITCH_WALK_THRESHOLD = 3;    
const int PITCH_SPRINT_THRESHOLD = 5; 
const int PITCH_BACKUP_THRESHOLD = 10; 

// -- FlexiForce Aim-Mode "Clutch" (on A1) --
const int FLEX_PIN = A1; 
int FLEX_AIM_THRESHOLD = 700; 

// -- MPU6050 "Movement Mode" Mouse Control (Constant Speed) --
const float ROLL_DEADZONE_RIGHT = 6.0; 
const int MOUSE_SPEED_RIGHT = 8; 
const float ROLL_DEADZONE_LEFT = 12.0; 
const int MOUSE_SPEED_LEFT = 8; 

// -- MPU6050 "Fine-Aim Mode" Mouse Control (Proportional) --
const float AIM_ROLL_DEADZONE_LEFT = 7.0; 
float mouseMultiplierAimLeft = 0.3;       
const float AIM_ROLL_DEADZONE_RIGHT = 7.0; 
float mouseMultiplierAimRight = 0.3;       
const float AIM_PITCH_DEADZONE_UP = 3.0;   
float mouseMultiplierAimUp = 0.3;          
const float AIM_PITCH_DEADZONE_DOWN = 10.0; 
float mouseMultiplierAimDown = 0.2;
const int AIM_MODE_COOLDOWN_MS = 500;

// -- MPU6050 Complementary Filter --
float complementaryFactor = 0.98;

// -- Sip & Puff Control (MPX5050 Sensor on A0) --
const int SIPPUFF_PIN = A0;              
const int HARD_PUFF_THRESHOLD = 30;   
const int SOFT_PUFF_THRESHOLD = 15;    
const int SIP_THRESHOLD = -5;         
const int NEUTRAL_DEADZONE = 5;       
const int EVALUATION_PERIOD = 10;     

// --- State Tracking (All states from both codes) ---

// MPU6050 States
float stableAnglePitch = 0.0;
float stableAngleRoll = 0.0;
unsigned long lastUpdateTime = 0;
struct KeyStates {
  bool w = false, s = false, left_ctrl = false;
  bool left_shift = false; // (Not used in V9.1, but here for completeness)
};
KeyStates keyStates;
float neutralPitchOffset = 0.0; 
float neutralRollOffset = 0.0;  

bool aimModeWasEngaged = false;
unsigned long aimModeReleaseTime = 0;

// Sip & Puff States
int sipPuffBaseline = 0;
bool sipActionTaken = false; 
bool isLeftClickHeld = false;
bool isJumpHeld = false;
bool isEvaluatingPuff = false;
int puffMeasurementCounter = 0;


// --- Helper Functions (From MPU code) ---
void handleKeyPress(uint8_t key, bool shouldBePressed, bool &isPressed) {
  if (shouldBePressed && !isPressed) {
    Keyboard.press(key); isPressed = true;
  } else if (!shouldBePressed && isPressed) {
    Keyboard.release(key); isPressed = false;
  }
}
float calculateAngleDifference(float currentAngle, float targetAngle) {
  float diff = currentAngle - targetAngle;
  while (diff <= -180) diff += 360;
  while (diff > 180) diff -= 360;
  return diff;
}

// --- setup() (Combined) ---
void setup() {
  Serial.begin(115200); 

  // --- 1. MPU6050 Initialization ---
  if (!mpu.begin()) {
    Serial.print("Ooops, no MPU6050 detected...");
    while (1);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- 2. Initialize Keyboard & Mouse ---
  Keyboard.begin();
  Mouse.begin();

  // --- 3. Calibrate Sip & Puff Pressure Sensor (from V2.5) ---
  Serial.println("Calibrating Pressure Sensor (Sip/Puff)...");
  long pressureSum = 0;
  for (int i = 0; i < 500; i++) {
    pressureSum += analogRead(SIPPUFF_PIN);
    delay(2);
  }
  sipPuffBaseline = pressureSum / 500;
  Serial.print("Sip/Puff Baseline: ");
  Serial.println(sipPuffBaseline);

  // --- 4. Calibrate "Neutral Pitch and Roll Values" (from V9.1) ---
  Serial.println("Calibrating \"Neutral Pitch and Roll Values\"...");
  delay(2000); 
  sensors_event_t a, g, temp; 
  float pitchSum = 0;
  float rollSum = 0;
  int calibrationSamples = 50;
  for (int i = 0; i < calibrationSamples; i++) {
     mpu.getEvent(&a, &g, &temp); 
     pitchSum += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
     rollSum += atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
     delay(10);
  }
  neutralPitchOffset = pitchSum / calibrationSamples;
  neutralRollOffset = rollSum / calibrationSamples;
  stableAnglePitch = neutralPitchOffset;
  stableAngleRoll = neutralRollOffset;

  Serial.print("Neutral Pitch Offset (W/S Control): ");
  Serial.println(neutralPitchOffset);
  Serial.print("Neutral Roll Offset (Mouse Control): ");
  Serial.println(neutralRollOffset);
  
  // --- 5. Initialize MPU Filter Timer ---
  lastUpdateTime = millis();
  
  Serial.println("MPU6050 + SipPuff Controller ready.");
}


void loop() {
  // --- BLOCK 1: Sip & Puff Logic (from V2.5) ---
  int currentSipPuffPressure = analogRead(SIPPUFF_PIN);
  int sipPuffDelta = currentSipPuffPressure - sipPuffBaseline;
  bool sipPuff_isNeutral = abs(sipPuffDelta) < NEUTRAL_DEADZONE;
  bool sipPuff_isHardPuff = sipPuffDelta > HARD_PUFF_THRESHOLD;
  bool sipPuff_isSip = sipPuffDelta < SIP_THRESHOLD;
  bool sipPuff_isSoftPuffRange = (sipPuffDelta > SOFT_PUFF_THRESHOLD) && (sipPuffDelta <= HARD_PUFF_THRESHOLD);

  if (sipPuff_isNeutral) {
    if (sipActionTaken) { sipActionTaken = false; }
    if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
    isEvaluatingPuff = false; puffMeasurementCounter = 0;
  }
  else if (sipPuff_isSip && !sipActionTaken) {
    Mouse.click(MOUSE_RIGHT);
    sipActionTaken = true; 
    isEvaluatingPuff = false; puffMeasurementCounter = 0;
    if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
  }
  else if (!sipActionTaken) {
    if (sipPuff_isHardPuff) { 
      if (!isJumpHeld) {
        Keyboard.press(' '); isJumpHeld = true;
      }
      isEvaluatingPuff = false; puffMeasurementCounter = 0;
      if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    }
    else if (sipPuff_isSoftPuffRange) { 
      if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
      if (!isLeftClickHeld) { 
        if (!isEvaluatingPuff) {
          isEvaluatingPuff = true; puffMeasurementCounter = 0;
        }
        else { 
          puffMeasurementCounter++;
          if (puffMeasurementCounter >= EVALUATION_PERIOD) {
            Mouse.press(MOUSE_LEFT); isLeftClickHeld = true;
            isEvaluatingPuff = false;
          }
        }
      }
    }
    else { 
      if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
      if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
      if (isEvaluatingPuff) { isEvaluatingPuff = false; puffMeasurementCounter = 0; }
    }
  }
  // --- Sip & Puff Logic End ---


  // --- BLOCK 2: MPU6050 Logic (from V9.1) ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // MPU Complementary Filter
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0; 
  lastUpdateTime = currentTime;
  float accelAnglePitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  float accelAngleRoll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  stableAnglePitch = complementaryFactor * (stableAnglePitch + (g.gyro.y * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAnglePitch);
  stableAngleRoll  = complementaryFactor * (stableAngleRoll  + (g.gyro.x * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAngleRoll);

  // Calculate Relative Displacements
  float relativePitch = stableAnglePitch - neutralPitchOffset; 
  float relativeRoll = stableAngleRoll - neutralRollOffset; 

  // Read FlexiForce Sensor (Mode Switch)
  int flexValue = analogRead(FLEX_PIN);
  bool aimModeIsEngaged = (flexValue > FLEX_AIM_THRESHOLD);

  // Initialize control variables
  bool should_w_be_pressed = false;
  bool should_s_be_pressed = false;
  bool should_ctrl_be_pressed = false;  
  
  if (!aimModeIsEngaged && aimModeWasEngaged) {
    // We just switched from AIM -> MOVEMENT
    // Record the time we let go.
    aimModeReleaseTime = millis();
  }
  aimModeWasEngaged = aimModeIsEngaged;

  int mouse_vx = 0;
  int mouse_vy = 0; 
  
  // MPU Mode-Switching Logic
  if (!aimModeIsEngaged) {
    // --- MODE 1: MOVEMENT ---
    // 1. Keyboard Movement Control IS ACTIVE
    unsigned long timeSinceAimRelease = millis() - aimModeReleaseTime;

    if (relativePitch > PITCH_SPRINT_THRESHOLD) { 
      should_w_be_pressed = true; 
      should_ctrl_be_pressed = true;
    } 
    else if (relativePitch > PITCH_WALK_THRESHOLD) {
      should_w_be_pressed = true; 
    }
    else if (relativePitch < -PITCH_BACKUP_THRESHOLD) {
      should_s_be_pressed = true; 
    }

    // 2. Constant-Speed Mouse X Control IS ACTIVE
    if (timeSinceAimRelease > AIM_MODE_COOLDOWN_MS) {
      if (relativeRoll > ROLL_DEADZONE_RIGHT) {
        mouse_vx = MOUSE_SPEED_RIGHT; 
      } 
      else if (relativeRoll < -ROLL_DEADZONE_LEFT) {
        mouse_vx = -MOUSE_SPEED_LEFT;
      }
    }
    
    handleKeyPress(KEY_LEFT_SHIFT, false, keyStates.left_shift);
  } 
  else {
    // --- MODE 2: FINE-AIM ---
    // 1. Keyboard Movement Control IS DISABLED
    
    // 2. Proportional Mouse X/Y Control IS ACTIVE
    if (relativeRoll > AIM_ROLL_DEADZONE_RIGHT) { // Aim Right
      mouse_vx = (int)( (relativeRoll - AIM_ROLL_DEADZONE_RIGHT) * mouseMultiplierAimRight );
    } 
    else if (relativeRoll < -AIM_ROLL_DEADZONE_LEFT) { // Aim Left
      mouse_vx = (int)( (relativeRoll + AIM_ROLL_DEADZONE_LEFT) * mouseMultiplierAimLeft );
    }
    if (relativePitch > AIM_PITCH_DEADZONE_UP) { // Aim Up
      mouse_vy = (int)( (relativePitch - AIM_PITCH_DEADZONE_UP) * mouseMultiplierAimUp );
    }
    else if (relativePitch < -AIM_PITCH_DEADZONE_DOWN) { // Aim Down
      mouse_vy = (int)( (relativePitch + AIM_PITCH_DEADZONE_DOWN) * mouseMultiplierAimDown );
    }
    
    handleKeyPress(KEY_LEFT_SHIFT, false, keyStates.left_shift);
  }
  
  // Update All MPU Keys
  handleKeyPress('w', should_w_be_pressed, keyStates.w);
  handleKeyPress('s', should_s_be_pressed, keyStates.s);
  handleKeyPress(KEY_LEFT_CTRL, should_ctrl_be_pressed, keyStates.left_ctrl);
  
  // Move The Mouse
  if (mouse_vx != 0 || mouse_vy != 0) {
    Mouse.move(mouse_vx, mouse_vy, 0); 
  }
  
  // --- Unified Serial Monitor (Debug Output) ---
  Serial.print("Mode: ");
  if (aimModeIsEngaged) {
    Serial.print("AIMING");
  } else {
    Serial.print("MOVEMENT");
  }
  Serial.print(" | Pitch(W/S): "); Serial.print(relativePitch, 1); 
  Serial.print(" | Roll(Mouse): "); Serial.print(relativeRoll, 1);
  Serial.print(" | M_VX: "); Serial.print(mouse_vx); 
  Serial.print(" | M_VY: "); Serial.print(mouse_vy);
  Serial.print(" | Flex: "); Serial.print(flexValue);
  
  // Add Sip/Puff debug info
  String sipPuffKeysStr = "";
  if (isLeftClickHeld) sipPuffKeysStr += "LClick ";
  if (isJumpHeld) sipPuffKeysStr += "Jump ";
  if (sipActionTaken) sipPuffKeysStr += "RClick(Lock) ";
  if (isEvaluatingPuff) sipPuffKeysStr += "Eval ";
  if (sipPuffKeysStr.length() == 0 && !sipPuff_isNeutral) sipPuffKeysStr = "Wait";
  else if (sipPuffKeysStr.length() == 0) sipPuffKeysStr = "Idle";
  Serial.print(" | Sip/Puff: " + sipPuffKeysStr + "(D:" + String(sipPuffDelta) + ")");
  
  Serial.println(); 
    
  delay(8); 
}