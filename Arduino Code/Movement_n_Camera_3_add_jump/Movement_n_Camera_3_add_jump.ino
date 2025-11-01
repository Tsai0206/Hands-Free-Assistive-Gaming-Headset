/*
 * MPU6050 Hybrid Controller (Keyboard + Clutch Mouse + Jump)
 * Version: 5.3 (Jump Feature Added)
 * Description: This version adds a jump function (Space Bar).
 * A jump is triggered when the FlexiForce sensor is pressed hard,
 * exceeding a separate JUMP_THRESHOLD. This action is independent
 * and can be performed while walking, sprinting, and looking around.
 *
 * Code Enhanced by: Code Companion
 * Current Time: Saturday, October 11, 2025 at 9:00:42 AM CST
 * Location: Taiwan
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

Adafruit_MPU6050 mpu;

// --- Tweakable Parameters (All in one place) ---
// -- Keyboard Control (State-based) --
const int WALK_THRESHOLD = 10;
const int SPRINT_THRESHOLD = 30;

// -- Mouse "Clutch" and Jump Control --
const int flexPin = A1;
int flexThreshold = 850;        // Pressure threshold to activate mouse view
const int JUMP_THRESHOLD = 1000; // --- NEW --- Pressure threshold to trigger a jump

float mouseMultiplierX = 60.0;
float mouseMultiplierY = 20.0;
float deadzoneThreshold = 0.1;

// --- State Tracking ---
struct KeyStates {
  bool w = false;
  bool left_ctrl = false;
};
KeyStates keyStates;

float gyroX_offset = 0.0;
float gyroZ_offset = 0.0;

bool clutchWasEngaged = false;
// --- NEW --- A variable to track the jump state to ensure one jump per press
bool jumpWasTriggeredInThisPress = false;

void handleKeyPress(uint8_t key, bool shouldBePressed, bool &isPressed) {
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
  if (!mpu.begin()) { /* ... error handling ... */ }
  Serial.println("MPU6050 Found!");
  
  // Gyro calibration
  Serial.println("Calibrating Gyroscope... Do not move the sensor.");
  delay(1000);
  // ... (calibration loop remains the same) ...
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
  Serial.println("Initial Calibration Complete.");

  Keyboard.begin();
  Mouse.begin();
  
  Serial.println("Hybrid Controller is ready.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Keyboard Movement Control (Always On) ---
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  // ... (walk/sprint logic remains the same) ...
  float abs_roll = abs(roll);
  bool should_w_be_pressed = false;
  bool should_ctrl_be_pressed = false;
  String moveState = "IDLE";
  if (abs_roll > SPRINT_THRESHOLD) {
    moveState = "SPRINTING";
    should_w_be_pressed = true;
    should_ctrl_be_pressed = true;
  } else if (abs_roll > WALK_THRESHOLD) {
    moveState = "WALKING";
    should_w_be_pressed = true;
    should_ctrl_be_pressed = false;
  }
  handleKeyPress('w', should_w_be_pressed, keyStates.w);
  handleKeyPress(KEY_LEFT_CTRL, should_ctrl_be_pressed, keyStates.left_ctrl);

  // --- Read FlexiForce sensor for both Clutch and Jump ---
  int flexValue = analogRead(flexPin);

  // --- INDEPENDENT JUMP CONTROL LOGIC ---
  // This logic runs independently and can override/accompany other actions.
  bool jumpThresholdMet = (flexValue > JUMP_THRESHOLD);

  if (jumpThresholdMet && !jumpWasTriggeredInThisPress) {
    // The pressure just crossed the jump threshold for the first time.
    // We execute a quick press-and-release of the space bar.
    Keyboard.press(' ');
    delay(25); // A very short delay to ensure the key press is registered
    Keyboard.release(' ');
    jumpWasTriggeredInThisPress = true; // Mark that we've jumped for this press
  } else if (!jumpThresholdMet) {
    // Once the pressure drops below the threshold, we reset the trigger,
    // allowing for the next jump.
    jumpWasTriggeredInThisPress = false;
  }
  
  // --- Clutch-activated Mouse Camera Control ---
  bool clutchIsEngaged = (flexValue > flexThreshold);
  if (clutchIsEngaged && !clutchWasEngaged) {
    // Dynamic re-calibration logic remains the same
    Serial.println("--- MOUSE CLUTCH ENGAGED: RE-CALIBRATING ZERO POINT ---");
    sensors_event_t current_a, current_g, current_temp;
    mpu.getEvent(&current_a, &current_g, &current_temp);
    gyroX_offset = current_g.gyro.x;
    gyroZ_offset = current_g.gyro.z;
  }
  if (clutchIsEngaged) {
    // Mouse logic remains the same
    float calibrated_gx = g.gyro.x - gyroX_offset;
    float calibrated_gz = g.gyro.z - gyroZ_offset;
    int mouse_vx = 0, mouse_vy = 0;
    if (abs(calibrated_gz) > deadzoneThreshold) { mouse_vx = -(int)(calibrated_gz * mouseMultiplierX); }
    if (abs(calibrated_gx) > deadzoneThreshold) { mouse_vy = (int)(calibrated_gx * mouseMultiplierY); }
    if (mouse_vx != 0 || mouse_vy != 0) { Mouse.move(mouse_vx, mouse_vy); }
  }
  clutchWasEngaged = clutchIsEngaged;

  // --- Unified Serial Monitor Output ---
  Serial.print("Move State: " + moveState + " (Roll: " + String(roll, 1) + ")");
  Serial.print(" | Mouse: ");
  if (clutchIsEngaged) { Serial.print("ON"); } else { Serial.print("OFF"); }
  Serial.print(" | Jump Ready: ");
  if (jumpWasTriggeredInThisPress) { Serial.print("NO"); } else { Serial.print("YES"); }
  Serial.println(" | Flex: " + String(flexValue));

  delay(20);
}