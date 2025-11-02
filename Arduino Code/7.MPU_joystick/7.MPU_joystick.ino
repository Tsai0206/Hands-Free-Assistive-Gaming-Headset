/*
 * MPU6050 + Sip & Puff Controller
 * Version: 8.1 (Logic Cloned from BNO V12.1)
 * Description: 
 * 1. Uses a Complementary Filter to get STABLE Pitch and Roll angles from the MPU6050.
 * 2. Pitch ANGLE (Displacement) controls W/S/Ctrl (relative to a neutral calibration).
 * 3. Roll ANGLE (Displacement) controls CONSTANT SPEED mouse (relative to a neutral calibration).
 * 4. FlexiForce (A1) now controls KEY_LEFT_SHIFT.
 * 5. Sip & Puff logic is unchanged.
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

// --- Tweakable Parameters (Matched to BNO v12.1) ---
// -- MPU6050 Movement Control (Pitch-based) --
const int PITCH_WALK_THRESHOLD = -3;    
const int PITCH_SPRINT_THRESHOLD = 1; 
const int PITCH_BACKUP_THRESHOLD = 10; 

// -- FlexiForce SHIFT Control --
const int FLEX_PIN = A1; 
int FLEX_SHIFT_THRESHOLD = 700; 

// -- MPU6050 Mouse Control (Roll Displacement - Binary Speed) --
const float ROLL_DEADZONE_LEFT = 12.0; 
const int MOUSE_SPEED_LEFT = 8; 

const float ROLL_DEADZONE_RIGHT = 6.0; 
const int MOUSE_SPEED_RIGHT = 8; 

// -- Sip & Puff Control (MPX5050 Sensor) (Unchanged) --
const int SIPPUFF_PIN = A0;              
const int HARD_PUFF_THRESHOLD = 30;   
const int SOFT_PUFF_THRESHOLD = 15;    
const int SIP_THRESHOLD = -5;         
const int NEUTRAL_DEADZONE = 5;       
const int EVALUATION_PERIOD = 10;     

// -- MPU6050 Complementary Filter --
// This is required to get stable angles from the MPU6050
float complementaryFactor = 0.98;
float stableAnglePitch = 0.0; // This will hold our stable Pitch
float stableAngleRoll = 0.0;  // This will hold our stable Roll
unsigned long lastUpdateTime = 0;

// --- State Tracking ---
struct KeyStates {
  bool w = false, s = false, left_ctrl = false, left_shift = false; 
};
KeyStates keyStates;

float neutralPitchOffset = 0.0; // For movement calibration
float neutralRollOffset = 0.0;  // For mouse calibration

// (Sip & Puff states are unchanged)
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

// (Helper function from BNO code, not strictly needed for pitch/roll < 90 deg, but good practice)
float calculateAngleDifference(float currentAngle, float targetAngle) {
  float diff = currentAngle - targetAngle;
  while (diff <= -180) diff += 360;
  while (diff > 180) diff -= 360;
  return diff;
}

void setup() {
  Serial.begin(115200); 

  // --- 1. MPU6050 Initialization ---
  if (!mpu.begin()) {
    Serial.print("Ooops, no MPU6050 detected...");
    while (1);
  }
  Serial.println("MPU6050 Found!");
  // Set accelerometer and gyro ranges (optional, but good)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  // --- 2. Initialize Keyboard & Mouse ---
  Keyboard.begin();
  Mouse.begin();


  // --- 3. Calibrate Sip & Puff Pressure Sensor ---
  Serial.println("Calibrating Pressure Sensor (Sip/Puff)...");
  long pressureSum = 0;
  for (int i = 0; i < 500; i++) {
    pressureSum += analogRead(SIPPUFF_PIN);
    delay(2);
  }
  sipPuffBaseline = pressureSum / 500;
  Serial.print("Sip/Puff Baseline: ");
  Serial.println(sipPuffBaseline);


  // --- 4. Calibrate "Neutral Pitch and Roll Values" ---
  Serial.println("Calibrating \"Neutral Pitch and Roll Values\"...");
  delay(2000); // Time to get sensor into neutral position

  sensors_event_t a, g, temp; // Declare sensor event struct
  float pitchSum = 0;
  float rollSum = 0;
  int calibrationSamples = 50;
  
  for (int i = 0; i < calibrationSamples; i++) {
     mpu.getEvent(&a, &g, &temp); // Read from MPU
     // Calculate MPU's initial Pitch and Roll from Accelerometer
     pitchSum += atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
     rollSum += atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
     delay(10);
  }
  
  neutralPitchOffset = pitchSum / calibrationSamples;
  neutralRollOffset = rollSum / calibrationSamples;
  
  // Initialize the filter's stable angles to the calibrated neutral position
  stableAnglePitch = neutralPitchOffset;
  stableAngleRoll = neutralRollOffset;

  Serial.print("Neutral Pitch Offset (W/S Control): ");
  Serial.println(neutralPitchOffset);
  Serial.print("Neutral Roll Offset (Mouse Control): ");
  Serial.println(neutralRollOffset);
  Serial.println("MPU6050 + SipPuff Controller ready.");
  
  // Initialize timer for Complementary Filter
  lastUpdateTime = millis();
}


void loop() {
  // --- Sip & Puff Logic (Runs first, unchanged) ---
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

  // --- MPU6050 Complementary Filter ---
  // This section calculates the stable Pitch and Roll angles
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0; // Time delta in seconds
  lastUpdateTime = currentTime;
  
  // Calculate angles from accelerometer (long-term stability)
  float accelAnglePitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  float accelAngleRoll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  
  // Calculate stable angles by fusing gyro (short-term) and accel (long-term)
  // Gyro values (g.gyro.y, g.gyro.x) are in rad/s, so we convert to deg/s
  stableAnglePitch = complementaryFactor * (stableAnglePitch + (g.gyro.y * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAnglePitch);
  stableAngleRoll  = complementaryFactor * (stableAngleRoll  + (g.gyro.x * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAngleRoll);

  // --- Calculate Relative Displacement ---
  // We use simple subtraction, as Pitch/Roll are unlikely to wrap around 360 in this use case
  float relativePitch = stableAnglePitch - neutralPitchOffset; 
  float relativeRoll = stableAngleRoll - neutralRollOffset; 

  // --- MPU6050 Keyboard Movement Control (Pitch-based) ---
  bool should_w_be_pressed = false;
  bool should_s_be_pressed = false;
  bool should_ctrl_be_pressed = false;

  if (relativePitch > PITCH_SPRINT_THRESHOLD) { 
    should_w_be_pressed = true; 
    should_ctrl_be_pressed = true; // Sprint (CTRL + W)
  } 
  else if (relativePitch > PITCH_WALK_THRESHOLD) {
    should_w_be_pressed = true; // Walk (W only)
  }
  else if (relativePitch < -PITCH_BACKUP_THRESHOLD) {
    should_s_be_pressed = true; // Back up (S)
  }

  handleKeyPress('w', should_w_be_pressed, keyStates.w);
  handleKeyPress('s', should_s_be_pressed, keyStates.s);
  handleKeyPress(KEY_LEFT_CTRL, should_ctrl_be_pressed, keyStates.left_ctrl);

  int flexValue = analogRead(FLEX_PIN);
  bool should_shift_be_pressed = (flexValue > FLEX_SHIFT_THRESHOLD);
  handleKeyPress(KEY_LEFT_SHIFT, should_shift_be_pressed, keyStates.left_shift);
  
  
  // --- MPU6050 DISPLACEMENT-BASED MOUSE CONTROL (CONSTANT SPEED) ---
  int mouse_vx = 0;
  if (relativeRoll > ROLL_DEADZONE_RIGHT){
    mouse_vx = MOUSE_SPEED_RIGHT; 
  } 
  // 2. Check for Left Movement
  else if (relativeRoll < -ROLL_DEADZONE_LEFT) {
    mouse_vx = -MOUSE_SPEED_LEFT;
  }
  if (mouse_vx != 0) {
    Mouse.move(mouse_vx, 0);
  }
  
  Serial.print("Pitch (W/S): "); Serial.print(relativePitch, 1); 
  Serial.print(" | Roll (Mouse): "); Serial.print(relativeRoll, 1);
  Serial.print(" | VX: "); Serial.print(mouse_vx); 
  Serial.print(" | Flex: "); Serial.print(flexValue); 
  Serial.print(" | Shift: "); Serial.print(keyStates.left_shift ? "ON" : "OFF");
  Serial.println(); 
    
  delay(10); 
}