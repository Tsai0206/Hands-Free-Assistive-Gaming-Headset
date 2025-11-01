/*
 * BNO055 + Sip & Puff Complete Controller
 * Version: 10.1 (SipPuff Logic Fix & Debug Output)
 * Description: 
 * 1. (FIX) Corrected Soft Puff threshold, eliminating the logical gap (3~7).
 * 2. (FIX) Increased Soft Puff evaluation period (EVALUATION_PERIOD) to 100ms for stability.
 * 3. (NEW) Uncommented all Serial.print lines in the loop() for full debugging.
 *
 * Code Integrated by: Code Companion
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Keyboard.h>
#include <Mouse.h>

// Using I2C address 0x29
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// --- Tweakable Parameters ---
// -- BNO055 Movement Control (based on Yaw - Euler X) --
const int YAW_WALK_THRESHOLD = 10;   // Degrees of left turn (Negative Yaw) to start walking
const int YAW_SPRINT_THRESHOLD = 15; // Degrees of left turn (Negative Yaw) to start sprinting
const int YAW_BACKUP_THRESHOLD = 10; // Degrees of right turn (Positive Yaw) to start backing up

// -- BNO055 Mouse Clutch Control (FlexiForce Sensor) --
const int FLEX_PIN = A1; // FlexiForce pin for mouse clutch
int FLEX_CLUTCH_THRESHOLD = 700;
float mouseMultiplierX = 1.2; 
float GYRO_DEADZONE = 0.2;  
const int MAX_MOUSE_SPEED_PER_FRAME = 25;

// -- (NEW) Sip & Puff Control (MPX5050 Sensor) --
const int SIPPUFF_PIN = A0;             // MPX5050 pressure sensor pin
const int HARD_PUFF_THRESHOLD = 30;   // Threshold for Hard Puff (Space Bar)
const int SOFT_PUFF_THRESHOLD = 5;    // (*** FIX ***) Threshold for Soft Puff (Left Click)
const int SIP_THRESHOLD = -5;         // Threshold for Sip (Right Click)
const int NEUTRAL_DEADZONE = 5;       // (*** FIX ***) Deadzone for Sip/Puff reset
const int EVALUATION_PERIOD = 10;     // (*** ADJUST ***) 10*10ms = 100ms evaluation time

// --- State Tracking ---
// BNO055 States
struct KeyStates {
  bool w = false, s = false, left_ctrl = false;
};
KeyStates keyStates;
float neutralYawOffset = 0.0;

// (NEW) Sip & Puff States
int sipPuffBaseline = 0;
bool sipActionTaken = false; // Only for the discrete Sip action
bool isLeftClickHeld = false;
bool isJumpHeld = false;
bool isEvaluatingPuff = false;
int puffMeasurementCounter = 0;


// --- Helper Functions ---
// (This function is now used for BNO's W, S, Ctrl)
void handleKeyPress(uint8_t key, bool shouldBePressed, bool &isPressed) {
  if (shouldBePressed && !isPressed) {
    Keyboard.press(key); isPressed = true;
  } else if (!shouldBePressed && isPressed) {
    Keyboard.release(key); isPressed = false;
  }
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


  // --- 2. BNO055 Full Sensor Calibration ---
  Serial.println("--- Full Sensor Calibration ---");
  Serial.println("Please pick up the sensor and draw figure-8s in the air");
  Serial.println("until all calibration statuses reach 3.");
  

  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  while (sys != 3) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    
    Serial.print("Calibration Status: ");
    Serial.print("System(SYS):"); Serial.print(sys, DEC);
    Serial.print(" Gyro(G):"); Serial.print(gyro, DEC);
    Serial.print(" Accel(A):"); Serial.print(accel, DEC);
    Serial.print(" Mag(M):"); Serial.println(mag, DEC);
    delay(100); 
  }
  Serial.println("\n--- Full Calibration Complete! ---");


  // --- 3. Initialize Keyboard and Mouse ---
  Keyboard.begin();
  Mouse.begin();


  // --- 4. (NEW) Calibrate Sip & Puff Pressure Sensor ---
  Serial.println("Calibrating pressure sensor (Sip/Puff)... Do not sip or puff.");
  long pressureSum = 0;
  for (int i = 0; i < 500; i++) {
    pressureSum += analogRead(SIPPUFF_PIN);
    delay(2);
  }
  sipPuffBaseline = pressureSum / 500;
  Serial.print("Sip/Puff Calibration Complete. Baseline: ");
  Serial.println(sipPuffBaseline);


  // --- 5. Calibrate "Neutral Yaw" ---
  Serial.println("Calibrating 'Neutral Yaw'... Please point the sensor forward.");
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

  Serial.print("Neutral Yaw Offset Calculated: ");
  Serial.println(neutralYawOffset);
  Serial.println("BNO055 + SipPuff Controller Ready.");
}

// BNO055 function to calculate the shortest angle difference
float calculateAngleDifference(float currentAngle, float targetAngle) {
  float diff = currentAngle - targetAngle;
  while (diff <= -180) diff += 360;
  while (diff > 180) diff -= 360;
  return diff;
}

void loop() {
  // Read BNO Calibration Status
  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  // --- (NEW) Sip & Puff Logic (Always runs at the top of the loop) ---
  int currentSipPuffPressure = analogRead(SIPPUFF_PIN);
  int sipPuffDelta = currentSipPuffPressure - sipPuffBaseline;

  // Define Sip/Puff states (using corrected thresholds)
  bool sipPuff_isNeutral = abs(sipPuffDelta) < NEUTRAL_DEADZONE;
  bool sipPuff_isHardPuff = sipPuffDelta > HARD_PUFF_THRESHOLD;
  bool sipPuff_isSip = sipPuffDelta < SIP_THRESHOLD;
  bool sipPuff_isSoftPuffRange = (sipPuffDelta > SOFT_PUFF_THRESHOLD) && (sipPuffDelta <= HARD_PUFF_THRESHOLD);

  // Sip/Puff Part 1: Handle Neutral State
  if (sipPuff_isNeutral) {
    if (sipActionTaken) { sipActionTaken = false; }
    if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
    isEvaluatingPuff = false; puffMeasurementCounter = 0;
  }
  // Sip/Puff Part 2: Handle Sip (Discrete Action)
  else if (sipPuff_isSip && !sipActionTaken) {
    Mouse.click(MOUSE_RIGHT);
    sipActionTaken = true; // Lock until returned to neutral
    isEvaluatingPuff = false; puffMeasurementCounter = 0;
    if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
  }
  // Sip/Puff Part 3: Handle Puffing (Continuous Actions) - Only if Sip is not active
  else if (!sipActionTaken) {
    if (sipPuff_isHardPuff) { // Priority 1: Hard Puff
      if (!isJumpHeld) {
        Keyboard.press(' '); isJumpHeld = true;
      }
      isEvaluatingPuff = false; puffMeasurementCounter = 0;
      if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    }
    else if (sipPuff_isSoftPuffRange) { // Priority 2: Soft Puff
      if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
      if (isLeftClickHeld) { /* Keep holding */ }
      else if (!isEvaluatingPuff) {
        isEvaluatingPuff = true; puffMeasurementCounter = 0;
      }
      else { // Evaluating
        puffMeasurementCounter++;
        if (puffMeasurementCounter >= EVALUATION_PERIOD) {
          Mouse.press(MOUSE_LEFT); isLeftClickHeld = true;
          isEvaluatingPuff = false;
        }
      }
    }
    else { // Priority 3: Outside action zones (but not neutral or sip)
           // (This block should now be avoided due to corrected thresholds)
      if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
      if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
      if (isEvaluatingPuff) { isEvaluatingPuff = false; puffMeasurementCounter = 0; }
    }
  }
  // --- Sip & Puff Logic END ---


  // --- BNO055 Logic (Executes based on calibration status) ---
  // --- BNO055 邏輯 (根據校準狀態執行) ---
if (gyro == 3 && accel == 3) {
    
    // --- BNO055 Core Logic ---
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    // --- BNO055 Keyboard Movement Control (Yaw-based) ---
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

    // --- Read FlexiForce Sensor (Clutch Only) ---
    int flexValue = analogRead(FLEX_PIN);
    bool clutchIsEngaged = (flexValue > FLEX_CLUTCH_THRESHOLD);

    // --- Clutch-activated Mouse Camera Control (using BNO055 Gyro X) ---
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

    // --- (NEW) Unified Serial Monitor (Uncommented) ---
    
    Serial.print("Move: " + moveState + " (Rel Yaw: " + String(relativeYaw, 1) + ")");
    Serial.print(" | Clutch: ");
    if (clutchIsEngaged) { Serial.print("ON"); } else { Serial.print("OFF"); }
    Serial.print(" (Flex: " + String(flexValue) + ")");

    // (NEW) Sip/Puff Debug Messages
    String sipPuffKeysStr = "";
    if (isLeftClickHeld) sipPuffKeysStr += "LClick "; if (isJumpHeld) sipPuffKeysStr += "Jump "; if (sipActionTaken) sipPuffKeysStr += "RClick(Lock) ";
    if (isEvaluatingPuff) sipPuffKeysStr += "Eval ";
    if (sipPuffKeysStr.length() == 0 && !sipPuff_isNeutral) sipPuffKeysStr = "Wait"; else if (sipPuffKeysStr.length() == 0) sipPuffKeysStr = "Idle";
    Serial.print(" | Sip/Puff: " + sipPuffKeysStr + "(D:" + String(sipPuffDelta) + ")");

    Serial.println();
    

  } 
  else {
    // --- (Safety Stop) BNO055 Calibration Lost ---
    Serial.println("!!! BNO Calibration Lost !!! Stopping all BNO actions...");

    // 1. Immediately release all BNO-controlled keys
    handleKeyPress('w', false, keyStates.w);
    handleKeyPress('s', false, keyStates.s);
    handleKeyPress(KEY_LEFT_CTRL, false, keyStates.left_ctrl);

    // 2. In-game feedback: "Stutter" the character
    Keyboard.press('w');
    delay(50); // Press for 50ms
    Keyboard.release('w');
  }


  // --- Unified Delay (based on BNO status) ---
  if (sys == 3) {
    delay(10); // Normal operation delay
  } else {
    delay(100); // Delay during re-calibration (allows "w" stutter feedback to execute)
  }
}