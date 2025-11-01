/*
 * Combined Controller: MPU6050 (Movement/View) + Sip & Puff (Actions)
 * Version: 8.0
 * Description: Integrates MPU6050-based movement and clutch-activated camera
 * view with MPX5050-based Sip & Puff controls for clicks and jump.
 * - MPU6050 Pitch -> W/S/Ctrl (Movement)
 * - MPU6050 Gyro X + FlexiForce Clutch (A1) -> Mouse X (Camera View)
 * - MPX5050 Pressure (A0):
 * - Soft Puff -> Hold Left Click
 * - Hard Puff -> Hold Space Bar
 * - Sip -> Tap Right Click
 * The FlexiForce Jump function has been removed.
 *
 * Code Integrated by: Code Companion
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

// --- Sensor Objects ---
Adafruit_MPU6050 mpu;

// --- Tweakable Parameters ---
// -- MPU6050 Movement Control --
const int WALK_THRESHOLD = 10;
const int SPRINT_THRESHOLD = 30;
const int BACKUP_THRESHOLD = 10;

// -- MPU6050 Mouse Clutch Control (FlexiForce Sensor) --
const int FLEX_PIN = A1;          // FlexiForce pin for mouse clutch
int FLEX_CLUTCH_THRESHOLD = 850;  // Pressure threshold to activate mouse view
float mouseMultiplierX = 20.0;    // Mouse sensitivity
float GYRO_DEADZONE = 0.1;        // Gyro deadzone for mouse

// -- Sip & Puff Control (MPX5050 Sensor) --
const int SIPPUFF_PIN = A0;             // MPX5050 pressure sensor pin
const int HARD_PUFF_THRESHOLD = 30;     // Threshold for Hard Puff (Space Bar)
const int SOFT_PUFF_THRESHOLD = 7;      // Threshold for Soft Puff (Left Click)
const int SIP_THRESHOLD = -5;           // Threshold for Sip (Right Click)
const int NEUTRAL_DEADZONE = 3;         // Deadzone for Sip/Puff reset
const int EVALUATION_PERIOD = 5;        // Measurements to confirm Soft Puff

// -- MPU6050 Complementary Filter --
float complementaryFactor = 0.98;
float angleX = 0; // Stable angle for mouse view reference (not directly used for movement in this version)
unsigned long lastUpdateTime = 0;

// --- State Tracking ---
// MPU6050 States
struct MpuKeyStates {
  bool w = false, s = false, left_ctrl = false;
};
MpuKeyStates mpuKeyStates;
bool clutchWasEngaged = false;

// Sip & Puff States
int sipPuffBaseline = 0;
bool sipActionTaken = false; // Only for the discrete Sip action
bool isLeftClickHeld = false;
bool isJumpHeld = false;
bool isEvaluatingPuff = false;
int puffMeasurementCounter = 0;

// --- Helper Functions ---
// handleKeyPress for MPU6050 movement keys
void handleMpuKeyPress(uint8_t key, bool shouldBePressed, bool &isPressed) {
  if (shouldBePressed && !isPressed) {
    Keyboard.press(key);
    isPressed = true;
  } else if (!shouldBePressed && isPressed) {
    Keyboard.release(key);
    isPressed = false;
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) { Serial.println("MPU6050 chip not found"); while (1) { delay(10); } }
  Serial.println("MPU6050 Found!");

  // Initialize Peripherals
  Keyboard.begin();
  Mouse.begin();

  // --- Calibrate Sip & Puff Sensor ---
  Serial.println("Calibrating pressure sensor (Sip/Puff)... Do not sip or puff.");
  long pressureSum = 0;
  for (int i = 0; i < 500; i++) {
    pressureSum += analogRead(SIPPUFF_PIN);
    delay(2);
  }
  sipPuffBaseline = pressureSum / 500;
  Serial.print("Sip/Puff Calibration Complete. Baseline: ");
  Serial.println(sipPuffBaseline);

  // Initialize timer for Complementary Filter
  lastUpdateTime = millis();

  Serial.println("Combined Controller iws ready.");
}

void loop() {
  // --- Read MPU6050 Data ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- MPU6050 Keyboard Movement Control (Always On) ---
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  bool should_w_be_pressed = false, should_s_be_pressed = false, should_ctrl_be_pressed = false;
  String moveState = "IDLE";
  if (pitch > SPRINT_THRESHOLD) {
    moveState = "SPRINTING";
    should_w_be_pressed = true; should_ctrl_be_pressed = true;
  } else if (pitch > WALK_THRESHOLD) {
    moveState = "WALKING";
    should_w_be_pressed = true;
  } else if (pitch < -BACKUP_THRESHOLD) {
    moveState = "BACKING UP";
    should_s_be_pressed = true;
  }
  handleMpuKeyPress('w', should_w_be_pressed, mpuKeyStates.w);
  handleMpuKeyPress('s', should_s_be_pressed, mpuKeyStates.s);
  handleMpuKeyPress(KEY_LEFT_CTRL, should_ctrl_be_pressed, mpuKeyStates.left_ctrl);

  // --- Read FlexiForce Sensor (Clutch Only) ---
  int flexValue = analogRead(FLEX_PIN);
  bool clutchIsEngaged = (flexValue > FLEX_CLUTCH_THRESHOLD);

  // --- MPU6050 Clutch-activated Mouse Camera Control ---
  // Complementary Filter Calculation
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = currentTime;
  float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  angleX = complementaryFactor * (angleX + (g.gyro.x * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAngleX);

  if (clutchIsEngaged) {
    float gyro_x_rad_s = g.gyro.x; // Use raw gyro for responsiveness
    int mouse_vx = 0;
    // Apply deadzone to raw gyro reading for mouse movement
    if (abs(gyro_x_rad_s) > GYRO_DEADZONE) {
      mouse_vx = (int)(gyro_x_rad_s * mouseMultiplierX);
    }
    if (mouse_vx != 0) {
      Mouse.move(mouse_vx, 0);
    }
  } else {
    // Reset stable angle when clutch is not engaged
    angleX = accelAngleX;
  }

  // --- Sip & Puff Control Logic (Independent) ---
  int currentSipPuffPressure = analogRead(SIPPUFF_PIN);
  int sipPuffDelta = currentSipPuffPressure - sipPuffBaseline;

  // Define Sip/Puff states
  bool sipPuff_isNeutral = abs(sipPuffDelta) < NEUTRAL_DEADZONE;
  bool sipPuff_isHardPuff = sipPuffDelta > HARD_PUFF_THRESHOLD;
  bool sipPuff_isSip = sipPuffDelta < SIP_THRESHOLD;
  bool sipPuff_isSoftPuffRange = (sipPuffDelta > SOFT_PUFF_THRESHOLD) && (sipPuffDelta <= HARD_PUFF_THRESHOLD);

  // Sip/Puff Part 1: Handle Neutral State
  if (sipPuff_isNeutral) {
    if (sipActionTaken) { sipActionTaken = false; /*Serial.println("Sip Reset");*/ }
    if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; /*Serial.println("LC Release");*/ }
    if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; /*Serial.println("Jump Release");*/ }
    isEvaluatingPuff = false; puffMeasurementCounter = 0;
  }
  // Sip/Puff Part 2: Handle Sip (Discrete)
  else if (sipPuff_isSip && !sipActionTaken) {
    //Serial.print("Right Click (Sip). Delta: "); Serial.println(sipPuffDelta);
    Mouse.click(MOUSE_RIGHT);
    sipActionTaken = true; // Lock until neutral
    isEvaluatingPuff = false; puffMeasurementCounter = 0;
    if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
  }
  // Sip/Puff Part 3: Handle Puffing (Continuous) - Only if Sip not active
  else if (!sipActionTaken) {
    if (sipPuff_isHardPuff) { // Priority 1: Hard Puff
      if (!isJumpHeld) {
        //Serial.print("Jump Hold START. Delta: "); Serial.println(sipPuffDelta);
        Keyboard.press(' '); isJumpHeld = true;
      }
      isEvaluatingPuff = false; puffMeasurementCounter = 0;
      if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; /*Serial.println("LC Auto-Release");*/ }
    }
    else if (sipPuff_isSoftPuffRange) { // Priority 2: Soft Puff
      if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; /*Serial.println("Jump Auto-Release");*/ }
      if (isLeftClickHeld) { /* Keep holding */ }
      else if (!isEvaluatingPuff) {
        isEvaluatingPuff = true; puffMeasurementCounter = 0; /*Serial.println("Evaluating Puff...");*/
      }
      else { // isEvaluatingPuff
        puffMeasurementCounter++;
        if (puffMeasurementCounter >= EVALUATION_PERIOD) {
          //Serial.print("LC Hold START. Delta: "); Serial.println(sipPuffDelta);
          Mouse.press(MOUSE_LEFT); isLeftClickHeld = true;
          isEvaluatingPuff = false;
        }
      }
    }
    else { // Priority 3: Not in any action zone (but not neutral or sip)
      if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; /*Serial.println("LC Release");*/ }
      if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; /*Serial.println("Jump Release");*/ }
      if (isEvaluatingPuff) { isEvaluatingPuff = false; puffMeasurementCounter = 0; /*Serial.println("Eval Canceled");*/ }
    }
  }

  // --- Unified Serial Monitor Output ---
  // (Optional - Can be very noisy, uncomment parts if needed for debugging)
  /*
  String mpuKeysStr = "";
  if (mpuKeyStates.w) mpuKeysStr += "W "; if (mpuKeyStates.s) mpuKeysStr += "S "; if (mpuKeyStates.left_ctrl) mpuKeysStr += "L_CTRL ";
  if (mpuKeysStr.length() == 0) mpuKeysStr = "None";
  Serial.print("MPU Keys: " + mpuKeysStr + " (P:" + String(pitch, 0) + ")");

  Serial.print(" | Clutch: "); if (clutchIsEngaged) Serial.print("ON"); else Serial.print("OFF");
  Serial.print("(F:" + String(flexValue)+")");
  Serial.print(" | View Angle:" + String(angleX, 1));


  String sipPuffKeysStr = "";
  if (isLeftClickHeld) sipPuffKeysStr += "LClick "; if (isJumpHeld) sipPuffKeysStr += "Jump "; if (sipActionTaken) sipPuffKeysStr += "RClick(Lock) ";
  if (isEvaluatingPuff) sipPuffKeysStr += "Eval ";
  if (sipPuffKeysStr.length() == 0 && !sipPuff_isNeutral) sipPuffKeysStr = "Wait"; else if (sipPuffKeysStr.length() == 0) sipPuffKeysStr = "Idle";
  Serial.print(" | Sip/Puff: " + sipPuffKeysStr + "(D:" + String(sipPuffDelta) + ")");

  Serial.println();
  */

  delay(10); // Keep delay short for responsiveness
}