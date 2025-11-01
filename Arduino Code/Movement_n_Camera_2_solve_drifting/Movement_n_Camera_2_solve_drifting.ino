/*
 * MPU6050 Hybrid Controller (Keyboard + Clutch Mouse)
 * Version: 5.2 (Dynamic Recalibration for Drift Fix)
 * Description: This version fixes the camera drift issue by dynamically 
 * re-calibrating the gyroscope's zero-point at the exact moment the 
 * FlexiForce clutch is pressed.
 *
 * Code Refactored by: Code Companion
 * Current Time: Saturday, October 11, 2025 at 8:46:26 AM CST
 * Location: Taiwan
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

Adafruit_MPU6050 mpu;

// --- Tweakable Parameters ---
const int WALK_THRESHOLD = 10;
const int SPRINT_THRESHOLD = 30;
const int flexPin = A1;
int flexThreshold = 850;
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

// --- NEW --- A variable to track the clutch state from the previous loop
bool clutchWasEngaged = false;

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

  if (!mpu.begin()) {
    Serial.println("MPU6050 chip not found");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  // Initial gyro calibration on startup
  Serial.println("Calibrating Gyroscope... Do not move the sensor.");
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

  // --- Clutch-activated Mouse Camera Control ---
  int flexValue = analogRead(flexPin);
  bool clutchIsEngaged = (flexValue > flexThreshold);

  // --- NEW LOGIC: DYNAMIC RE-CALIBRATION ---
  // Check if the clutch was just pressed in this loop iteration
  if (clutchIsEngaged && !clutchWasEngaged) {
    // This is the first moment the clutch is pressed.
    // We re-calibrate the gyro's "zero point" to the current readings.
    Serial.println("--- MOUSE CLUTCH ENGAGED: RE-CALIBRATING ZERO POINT ---");
    sensors_event_t current_a, current_g, current_temp;
    mpu.getEvent(&current_a, &current_g, &current_temp); // Get a fresh reading
    gyroX_offset = current_g.gyro.x;
    gyroZ_offset = current_g.gyro.z;
  }

  if (clutchIsEngaged) {
    // Mouse logic now uses the dynamically updated offsets
    float calibrated_gx = g.gyro.x - gyroX_offset;
    float calibrated_gz = g.gyro.z - gyroZ_offset;
    int mouse_vx = 0, mouse_vy = 0;
    if (abs(calibrated_gz) > deadzoneThreshold) {
      mouse_vx = -(int)(calibrated_gz * mouseMultiplierX);
    }
    if (abs(calibrated_gx) > deadzoneThreshold) {
      mouse_vy = (int)(calibrated_gx * mouseMultiplierY);
    }
    if (mouse_vx != 0 || mouse_vy != 0) {
      Mouse.move(mouse_vx, mouse_vy);
    }
  }
  
  // At the very end of the loop, update the state for the next cycle
  clutchWasEngaged = clutchIsEngaged;

  // --- Unified Serial Monitor Output ---
  Serial.print("Move State: " + moveState + " (Roll: " + String(roll, 1) + ")");
  Serial.print(" | Mouse Mode: ");
  if (clutchIsEngaged) {
    Serial.println("ON (Flex: " + String(flexValue) + ")");
  } else {
    Serial.println("OFF (Flex: " + String(flexValue) + ")");
  }

  delay(20);
}