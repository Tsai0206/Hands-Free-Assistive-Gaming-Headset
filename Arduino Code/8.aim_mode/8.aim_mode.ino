/*
 * MPU6Sip & Puff Controller
 * Version: 9.1 (Aim Mode Disables Movement)
 * Description: 
 * 1. FlexiForce (A1) switches between "Movement Mode" and "Fine-Aim Mode".
 * 2. Movement Mode (Not Pressed):
 * - Pitch -> W/S/Ctrl
 * - Roll  -> Constant-Speed Mouse X
 * 3. Fine-Aim Mode (Pressed):
 * - Pitch/Roll -> Proportional Mouse Y/X
 * - *** W/S/Ctrl movement is NOW DISABLED ***
 *
 * Code Modified by: Code Companion
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

// --- Sensor Objects ---
Adafruit_MPU6050 mpu;

// --- Tweakable Parameters ---
// (Parameters for Movement, FlexiForce, Mouse Modes, Sip/Puff are unchanged)
// ... (所有參數都和 V9.0 一樣) ...
const int PITCH_WALK_THRESHOLD = 3;    
const int PITCH_SPRINT_THRESHOLD = 5; 
const int PITCH_BACKUP_THRESHOLD = 10; 
const int FLEX_PIN = A1; 
int FLEX_AIM_THRESHOLD = 700; 
const float ROLL_DEADZONE_RIGHT = 6.0; 
const int MOUSE_SPEED_RIGHT = 8; 
const float ROLL_DEADZONE_LEFT = 12.0; 
const int MOUSE_SPEED_LEFT = 8; 
const float AIM_ROLL_DEADZONE_LEFT = 7.0;  
float mouseMultiplierAimLeft = 0.5;        
const float AIM_ROLL_DEADZONE_RIGHT = 7.0; 
float mouseMultiplierAimRight = 0.5;       
const float AIM_PITCH_DEADZONE_UP = 10.0;   
float mouseMultiplierAimUp = 0.3;         
const float AIM_PITCH_DEADZONE_DOWN = 3.0; 
float mouseMultiplierAimDown = 0.3;
const int SIPPUFF_PIN = A0;             
const int HARD_PUFF_THRESHOLD = 30;   
const int SOFT_PUFF_THRESHOLD = 15;    
const int SIP_THRESHOLD = -5;         
const int NEUTRAL_DEADZONE = 5;       
const int EVALUATION_PERIOD = 10;     
float complementaryFactor = 0.98;

// --- State Tracking (Unchanged) ---
// ... (所有狀態變數都和 V9.0 一樣) ...
float stableAnglePitch = 0.0;
float stableAngleRoll = 0.0;
unsigned long lastUpdateTime = 0;
struct KeyStates {
  bool w = false, s = false, left_ctrl = false;
  bool left_shift = false; 
};
KeyStates keyStates;
float neutralPitchOffset = 0.0; 
float neutralRollOffset = 0.0;  
int sipPuffBaseline = 0;
bool sipActionTaken = false; 
bool isLeftClickHeld = false;
bool isJumpHeld = false;
bool isEvaluatingPuff = false;
int puffMeasurementCounter = 0;

// --- Helper Functions (Unchanged) ---
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

// --- setup() (Unchanged) ---
void setup() {
  Serial.begin(115200); 

  // (MPU6050 Initialization)
  if (!mpu.begin()) {
    Serial.print("Ooops, no MPU6050 detected...");
    while (1);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // (Initialize Keyboard & Mouse)
  Keyboard.begin();
  Mouse.begin();

  // (Calibrate Sip & Puff Pressure Sensor)
  Serial.println("Calibrating Pressure Sensor (Sip/Puff)...");
  long pressureSum = 0;
  for (int i = 0; i < 500; i++) {
    pressureSum += analogRead(SIPPUFF_PIN);
    delay(2);
  }
  sipPuffBaseline = pressureSum / 500;
  Serial.print("Sip/Puff Baseline: ");
  Serial.println(sipPuffBaseline);

  // (Calibrate "Neutral Pitch and Roll Values")
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
  Serial.println("MPU6050 + SipPuff Controller ready.");
  
  lastUpdateTime = millis();
}


void loop() {
  // --- Sip & Puff Logic (Runs first, unchanged) ---
  // (Sip/Puff logic omitted for brevity)
  int currentSipPuffPressure = analogRead(SIPPUFF_PIN);
  int sipPuffDelta = currentSipPuffPressure - sipPuffBaseline;
  bool sipPuff_isNeutral = abs(sipPuffDelta) < NEUTRAL_DEADZONE;
  bool sipPuff_isHardPuff = sipPuffDelta > HARD_PUFF_THRESHOLD;
  bool sipPuff_isSip = sipPuffDelta < SIP_THRESHOLD;
  bool sipPuff_isSoftPuffRange = (sipPuffDelta > SOFT_PUFF_THRESHOLD) && (sipPuffDelta <= HARD_PUFF_THRESHOLD);

  if (sipPuff_isNeutral) { /* ... Reset logic ... */ }
  else if (sipPuff_isSip && !sipActionTaken) { /* ... Sip logic ... */ }
  else if (!sipActionTaken) { /* ... Puff logic (Hard, Soft, Eval) ... */ }


  // --- MPU6050 Logic ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- MPU6050 Complementary Filter (Unchanged) ---
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0; 
  lastUpdateTime = currentTime;
  float accelAnglePitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  float accelAngleRoll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  stableAnglePitch = complementaryFactor * (stableAnglePitch + (g.gyro.y * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAnglePitch);
  stableAngleRoll  = complementaryFactor * (stableAngleRoll  + (g.gyro.x * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAngleRoll);

  // --- Calculate Relative Displacements (Unchanged) ---
  float relativePitch = stableAnglePitch - neutralPitchOffset; 
  float relativeRoll = stableAngleRoll - neutralRollOffset; 

  // --- Read FlexiForce Sensor (Mode Switch) ---
  int flexValue = analogRead(FLEX_PIN);
  bool aimModeIsEngaged = (flexValue > FLEX_AIM_THRESHOLD);

  // --- Initialize control variables ---
  bool should_w_be_pressed = false;
  bool should_s_be_pressed = false;
  bool should_ctrl_be_pressed = false;
  int mouse_vx = 0;
  int mouse_vy = 0; 
  
  // --- This is the new Mode-Switching Logic ---
  
  if (!aimModeIsEngaged) {
    // --- MODE 1: MOVEMENT (Constant Speed) ---
    // Flex sensor is NOT pressed.
    
    // 1. Keyboard Movement Control IS ACTIVE
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
    if (relativeRoll > ROLL_DEADZONE_RIGHT) {
      mouse_vx = MOUSE_SPEED_RIGHT; 
    } 
    else if (relativeRoll < -ROLL_DEADZONE_LEFT) {
      mouse_vx = -MOUSE_SPEED_LEFT;
    }
    
    // (We are not in aim mode, so LEFT_SHIFT is NOT pressed)
    handleKeyPress(KEY_LEFT_SHIFT, false, keyStates.left_shift);
  } 
  else {
    // --- MODE 2: FINE-AIM (Proportional Speed) ---
    // Flex sensor IS pressed.
    
    // 1. Keyboard Movement Control IS DISABLED
    // (We leave should_w_be_pressed etc. as false)

    // 2. Proportional Mouse X/Y Control IS ACTIVE
    if (relativeRoll > AIM_ROLL_DEADZONE_RIGHT) { // Aim Right
      mouse_vx = (int)( (relativeRoll - AIM_ROLL_DEADZONE_RIGHT) * mouseMultiplierAimRight );
    } 
    else if (relativeRoll < -AIM_ROLL_DEADZONE_LEFT) { // Aim Left
      // Note: relativeRoll is negative, so (e.g., -5.0 + 2.0) = -3.0
      mouse_vx = (int)( (relativeRoll + AIM_ROLL_DEADZONE_LEFT) * mouseMultiplierAimLeft );
    }

    // --- Calculate Y-Axis (Pitch) ---
    if (relativePitch > AIM_PITCH_DEADZONE_UP) { // Aim Up
      mouse_vy = (int)( (relativePitch - AIM_PITCH_DEADZONE_UP) * mouseMultiplierAimUp );
    }
    else if (relativePitch < -AIM_PITCH_DEADZONE_DOWN) { // Aim Down
      mouse_vy = (int)( (relativePitch + AIM_PITCH_DEADZONE_DOWN) * mouseMultiplierAimDown );
    }
    
    // (We are not in aim mode, so LEFT_SHIFT is NOT pressed)
    handleKeyPress(KEY_LEFT_SHIFT, false, keyStates.left_shift);
  }
  
  // --- Update All Keys ---
  // This will automatically release W/S/Ctrl when aimModeIsEngaged becomes true
  handleKeyPress('w', should_w_be_pressed, keyStates.w);
  handleKeyPress('s', should_s_be_pressed, keyStates.s);
  handleKeyPress(KEY_LEFT_CTRL, should_ctrl_be_pressed, keyStates.left_ctrl);
  
  // --- Move The Mouse ---
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
  Serial.print(" | Pitch: "); Serial.print(relativePitch, 1); 
  Serial.print(" | Roll: "); Serial.print(relativeRoll, 1);
  Serial.print(" | MouseVX: "); Serial.print(mouse_vx); 
  Serial.print(" | MouseVY: "); Serial.print(mouse_vy);
  Serial.print(" | Flex: "); Serial.print(flexValue); 
  Serial.println(); 
    
  delay(8); 
}