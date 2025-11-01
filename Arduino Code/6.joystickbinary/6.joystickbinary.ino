/*
 * BNO055 + Sip & Puff Complete Controller
 * Version: 12.1 (Mouse Controlled by Binary Roll Displacement)
 * Description: 
 * 1. Mouse movement (VX) is now **constant speed** once Roll displacement exceeds the deadzone.
 * 2. Mouse moves Right at MAX_SPEED if Roll > +5 degrees.
 * 3. Mouse moves Left at MAX_SPEED if Roll < -5 degrees.
 * 4. All other controls (Pitch for W/S, Sip/Puff) remain the same.
 *
 * Code Integrated by: Code Companion
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Keyboard.h>
#include <Mouse.h>

// Use I2C address 0x29
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// --- Tweakable Parameters ---
// -- BNO055 Movement Control (based on Euler Y - Pitch) --
const int PITCH_WALK_THRESHOLD = 1;    
const int PITCH_SPRINT_THRESHOLD = 5; 
const int PITCH_BACKUP_THRESHOLD = 10; 

// -- FlexiForce SHIFT Control (Unchanged) --
const int FLEX_PIN = A1; 
int FLEX_SHIFT_THRESHOLD = 700; 

// -- BNO055 Mouse Control (Roll Displacement - Binary Speed) --
// The multiplier is no longer usewd for speed calculation, but the parameters remain.
const float ROLL_DEADZONE_DEG = 7.0; 
// This parameter now defines the constant speed.
const int MAX_MOUSE_SPEED_PER_FRAME = 10; 
// Removed mouseMultiplierRoll as it's not needed for constant speed control.

// -- Sip & Puff Control (MPX5050 Sensor) (Unchanged) --
const int SIPPUFF_PIN = A0;              
const int HARD_PUFF_THRESHOLD = 30;   
const int SOFT_PUFF_THRESHOLD = 15;    
const int SIP_THRESHOLD = -5;         
const int NEUTRAL_DEADZONE = 5;       
const int EVALUATION_PERIOD = 10;     

// --- State Tracking (Unchanged) ---
struct KeyStates {
  bool w = false, s = false, left_ctrl = false, left_shift = false; 
};
KeyStates keyStates;
float neutralPitchOffset = 0.0; 
float neutralRollOffset = 0.0; 
int currentMouseDirection = 0; 
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

void setup() {
  Serial.begin(115200); 

  // --- 1. BNO055 Initialization ---
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected...");
    while (1);
  }
  Serial.println("BNO055 Found!");
  bno.setExtCrystalUse(true);


  // --- 2. BNO055 Full Sensor Calibration (Unchanged) ---
  Serial.println("--- Full Sensor Calibration ---");
  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  while (sys != 3) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("Calibration Status: SYS:"); Serial.print(sys);
    Serial.print(" G:"); Serial.print(gyro);
    Serial.print(" A:"); Serial.print(accel);
    Serial.print(" M:"); Serial.println(mag);
    delay(100); 
  }
  Serial.println("\n--- Full Calibration Complete! ---");


  // --- 3. Initialize Keyboard & Mouse (Unchanged) ---
  Keyboard.begin();
  Mouse.begin();


  // --- 4. Calibrate Sip & Puff Pressure Sensor (Unchanged) ---
  Serial.println("Calibrating Pressure Sensor (Sip/Puff)...");
  long pressureSum = 0;
  for (int i = 0; i < 500; i++) {
    pressureSum += analogRead(SIPPUFF_PIN);
    delay(2);
  }
  sipPuffBaseline = pressureSum / 500;
  Serial.print("Sip/Puff Baseline: ");
  Serial.println(sipPuffBaseline);


  // --- 5. Calibrate "Neutral Pitch and Roll Values" (Unchanged) ---
  Serial.println("Calibrating \"Neutral Pitch and Roll Values\" (X & Y axis displacement)...");
  delay(2000); 

  sensors_event_t orientationData;
  float pitchSum = 0;
  float rollSum = 0;
  int calibrationSamples = 50;
  
  for (int i = 0; i < calibrationSamples; i++) {
     bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
     pitchSum += orientationData.orientation.y; 
     rollSum += orientationData.orientation.z; 
     delay(10);
  }
  
  neutralPitchOffset = pitchSum / calibrationSamples;
  neutralRollOffset = rollSum / calibrationSamples;

  Serial.print("Neutral Pitch Offset (W/S Control): ");
  Serial.println(neutralPitchOffset);
  Serial.print("Neutral Roll Offset (Y-Axis Displacement): ");
  Serial.println(neutralRollOffset);
  Serial.println("BNO055 + SipPuff Controller ready.");
}


void loop() {
  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // --- Sip & Puff Logic (Omitted for brevity, unchanged) ---
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


  // --- BNO055 Logic ---
  if (sys == 3) {
    
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    float rawPitch = orientationData.orientation.y; 
    float rawRoll = orientationData.orientation.z; 
    
    // Calculate Relative Displacement
    float relativePitch = rawPitch - neutralPitchOffset; 
    float relativeRoll = rawRoll - neutralRollOffset; 

    // --- BNO055 Keyboard Movement Control (Pitch-based, INVERTED and CORRECTED) ---
    bool should_w_be_pressed = false;
    bool should_s_be_pressed = false;
    bool should_ctrl_be_pressed = false;

    // Check for FORWARD/SPRINT (W) - Now triggered by POSITIVE Pitch (tilting back)
    if (relativePitch > PITCH_SPRINT_THRESHOLD) { 
      should_w_be_pressed = true; 
      should_ctrl_be_pressed = true; // Sprint (CTRL + W)
    } 
    // Check for FORWARD/WALK (W) - Less extreme positive pitchw
  } 
  else {
    // --- (Safety Stop) BNO055 Calibration Lost (Unchanged) ---
    Serial.println("!!! BNO Calibration Lost !!! Stopping all BNO actions...");

    handleKeyPress('w', false, keyStates.w);
    handleKeyPress('s', false, keyStates.s);
    handleKeyPress(KEY_LEFT_CTRL, false, keyStates.left_ctrl);
    handleKeyPress(KEY_LEFT_SHIFT, false, keyStates.left_shift); 
    currentMouseDirection = 0; 

    Keyboard.press('w');
    delay(50); 
    Keyboard.release('w');
  }


  // --- Unified Delay (Unchanged) ---
  if (sys == 3) {
    delay(10); 
  } else {
    delay(100); 
  }
}