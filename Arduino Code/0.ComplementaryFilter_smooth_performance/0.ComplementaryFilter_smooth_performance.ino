/*
 * MPU6050 Hybrid Controller (Remapped Controls)
 * Version: 7.0 (Complementary Filter for Drift Fix)
 * Description: This version implements a Complementary Filter to fuse
 * accelerometer and gyroscope data. This provides continuous, real-time
 * drift correction for the mouse camera view, fixing the drift issue.
 *
 * Code Enhanced by: Code Companion
 * Current Time: Friday, October 17, 2025 at 10:25:00 AM CST
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
const int BACKUP_THRESHOLD = 10;
const int flexPin = A1;
int flexThreshold = 850;
const int JUMP_THRESHOLD = 1000;
float mouseMultiplierX = 40.0;
float deadzoneThreshold = 0.1;

// --- NEW: Complementary Filter Variables ---
float complementaryFactor = 0.98; // 98% from Gyro, 2% from Accel. Determines how quickly it corrects drift.
float angleX = 0; // This will hold our stable, drift-corrected angle.
unsigned long lastUpdateTime = 0; // To calculate the time delta for integration

// --- State Tracking ---
struct KeyStates {
  bool w = false, s = false, left_ctrl = false;
};
KeyStates keyStates;

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
  Serial.begin(115220);
  if (!mpu.begin()) { Serial.println("MPU6050 chip not found"); while (1) { delay(10); } }
  Serial.println("MPU6050 Found!");

  // No initial gyro calibration needed for the filter, it self-corrects!
  
  Keyboard.begin();
  Mouse.begin();
  
  // Initialize the timer for the filter
  lastUpdateTime = millis();
  
  Serial.println("Hybrid Controller with Complementary Filter is ready.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Keyboard Movement Control (Always On) ---
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  bool should_w_be_pressed = false, should_s_be_pressed = false, should_ctrl_be_pressed = false;
  String moveState = "IDLE";

  if (pitch > SPRINT_THRESHOLD) {
    moveState = "SPRINTING";
    should_w_be_pressed = true;
    should_ctrl_be_pressed = true;
  } else if (pitch > WALK_THRESHOLD) {
    moveState = "WALKING";
    should_w_be_pressed = true;
  } else if (pitch < -BACKUP_THRESHOLD) {
    moveState = "BACKING UP";
    should_s_be_pressed = true;
  }
  handleKeyPress('w', should_w_be_pressed, keyStates.w);
  handleKeyPress('s', should_s_be_pressed, keyStates.s);
  handleKeyPress(KEY_LEFT_CTRL, should_ctrl_be_pressed, keyStates.left_ctrl);

  // --- Read FlexiForce sensor ---
  int flexValue = analogRead(flexPin);

  // --- INDEPENDENT JUMP CONTROL LOGIC ---
  bool jumpThresholdMet = (flexValue > JUMP_THRESHOLD);
  if (jumpThresholdMet && !jumpWasTriggeredInThisPress) {
    Keyboard.press(' '); delay(25); Keyboard.release(' ');
    jumpWasTriggeredInThisPress = true;
  } else if (!jumpThresholdMet) {
    jumpWasTriggeredInThisPress = false;
  }
  
  // --- Clutch-activated Mouse Camera Control (with Complementary Filter) ---
  bool clutchIsEngaged = (flexValue > flexThreshold);
  
  // --- NEW: Complementary Filter Calculation ---
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0; // Time delta in seconds
  lastUpdateTime = currentTime;

  // Calculate angle from accelerometer (long-term stability)
  float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;

  // Integrate gyro data to get angle change (short-term responsiveness)
  // Note: g.gyro.x is in radians/s, so we convert to degrees/s
  angleX = complementaryFactor * (angleX + (g.gyro.x * 180 / M_PI) * dt) + (1 - complementaryFactor) * (accelAngleX);
  
  if (clutchIsEngaged) {
    // We now use the gyroscope reading directly for mouse movement,
    // as the drift is handled by the filter's continuous correction of angleX.
    // The dynamic recalibration is no longer needed.
    float gyro_x_rad_s = g.gyro.x; // Use the raw gyro for responsiveness
    int mouse_vx = 0;
    if (abs(gyro_x_rad_s) > deadzoneThreshold) {
      mouse_vx = (int)(gyro_x_rad_s * mouseMultiplierX);
    }
    if (mouse_vx != 0) {
      Mouse.move(mouse_vx, 0);
    }
  } else {
    // When clutch is not engaged, reset the stable angle to the current tilt
    // This makes sure the camera view doesn't jump when you re-engage the clutch.
    angleX = accelAngleX;
  }

  // --- Unified Serial Monitor Output ---
  String pressedKeysStr = "";
  if (keyStates.w) pressedKeysStr += "W ";
  if (keyStates.s) pressedKeysStr += "S ";
  if (keyStates.left_ctrl) pressedKeysStr += "L_CTRL ";
  if (pressedKeysStr.length() == 0) pressedKeysStr = "None";

  Serial.print("Keys: " + pressedKeysStr + " (Pitch: " + String(pitch, 1) + ")");
  Serial.print(" | Mouse: ");
  if (clutchIsEngaged) { Serial.print("ON"); } else { Serial.print("OFF"); }
  Serial.print(" (Stable Angle X: " + String(angleX, 2) + ")"); // Show the drift-corrected angle
  Serial.print(" | Jump Ready: ");
  if (jumpWasTriggeredInThisPress) { Serial.print("NO"); } else { Serial.print("YES"); }
  Serial.println(" | Flex: " + String(flexValue));

  delay(10); // A slightly shorter delay can help the filter's accuracy
}