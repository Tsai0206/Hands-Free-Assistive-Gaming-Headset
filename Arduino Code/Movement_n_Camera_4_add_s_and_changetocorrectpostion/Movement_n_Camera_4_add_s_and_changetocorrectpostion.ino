/*
 * MPU6050 Hybrid Controller (Remapped Controls)
 * Version: 6.2 (Added Rotation Value to Output)
 * Description: This version modifies the serial output to always display
 * the calibrated gyroscope X-axis rotation value for easier tuning.
 *
 * Code Modified by: Code Companion
 * Current Time: Sunday, October 12, 2025 at 5:54:33 AM CST
 * Location: Taiwan
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

Adafruit_MPU6050 mpu;

// --- Tweakable Parameters (All in one place) ---
const int WALK_THRESHOLD = 10;
const int SPRINT_THRESHOLD = 30;
const int BACKUP_THRESHOLD = 10;
const int flexPin = A1;
int flexThreshold = 850;
const int JUMP_THRESHOLD = 1000;
float mouseMultiplierX = 40.0;
float deadzoneThreshold = 0.1;

// --- State Tracking ---
struct KeyStates {
  bool w = false;
  bool s = false;
  bool left_ctrl = false;
};
KeyStates keyStates;

float gyroX_offset = 0.0; 
bool clutchWasEngaged = false;
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
  if (!mpu.begin()) { Serial.println("MPU6050 chip not found"); while (1) { delay(10); } }
  Serial.println("MPU6050 Found!");
  
  // Gyro calibration (Now for X-axis only)
  Serial.println("Calibrating Gyroscope... Do not move the sensor.");
  delay(1000);
  const int calibrationSamples = 500;
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroX_offset += g.gyro.x; 
    delay(3);
  }
  gyroX_offset /= calibrationSamples; 
  Serial.println("Initial Calibration Complete.");

  Keyboard.begin();
  Mouse.begin();
  
  Serial.println("Hybrid Controller is ready.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Calculate all necessary values at the top ---
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  // This is the rotation value for the mouse view
  float calibrated_gx = g.gyro.x - gyroX_offset;

  // --- Keyboard Movement Control (Always On) ---
  bool should_w_be_pressed = false;
  bool should_s_be_pressed = false;
  bool should_ctrl_be_pressed = false;
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

  // --- Read FlexiForce sensor for both Clutch and Jump ---
  int flexValue = analogRead(flexPin);

  // --- INDEPENDENT JUMP CONTROL LOGIC ---
  bool jumpThresholdMet = (flexValue > JUMP_THRESHOLD);
  if (jumpThresholdMet && !jumpWasTriggeredInThisPress) {
    Keyboard.press(' ');
    delay(25);
    Keyboard.release(' ');
    jumpWasTriggeredInThisPress = true;
  } else if (!jumpThresholdMet) {
    jumpWasTriggeredInThisPress = false;
  }
  
  // --- Clutch-activated Mouse Camera Control (HORIZONTAL ONLY) ---
  bool clutchIsEngaged = (flexValue > flexThreshold);
  if (clutchIsEngaged && !clutchWasEngaged) {
    Serial.println("--- MOUSE CLUTCH ENGAGED: RE-CALIBRATING ZERO POINT ---");
    sensors_event_t current_a, current_g, current_temp;
    mpu.getEvent(&current_a, &current_g, &current_temp);
    gyroX_offset = current_g.gyro.x;
  }
  if (clutchIsEngaged) {
    // We now use the 'calibrated_gx' value calculated at the top
    int mouse_vx = 0;
    if (abs(calibrated_gx) > deadzoneThreshold) {
      mouse_vx = (int)(calibrated_gx * mouseMultiplierX);
    }
    if (mouse_vx != 0) {
      Mouse.move(mouse_vx, 0);
    }
  }
  clutchWasEngaged = clutchIsEngaged;

  // --- MODIFIED Unified Serial Monitor Output ---
  String pressedKeysStr = "";
  if (keyStates.w) pressedKeysStr += "W ";
  if (keyStates.s) pressedKeysStr += "S ";
  if (keyStates.left_ctrl) pressedKeysStr += "L_CTRL ";
  if (pressedKeysStr.length() == 0) pressedKeysStr = "None";

  Serial.print("Keys: " + pressedKeysStr + " (Pitch: " + String(pitch, 1) + ")");
  Serial.print(" | Mouse: ");
  if (clutchIsEngaged) { Serial.print("ON"); } else { Serial.print("OFF"); }
  
  // --- NEWLY ADDED ROTATION VALUE ---
  Serial.print(" (Rot. X: " + String(calibrated_gx, 2) + ")");

  Serial.print(" | Jump Ready: ");
  if (jumpWasTriggeredInThisPress) { Serial.print("NO"); } else { Serial.print("YES"); }
  Serial.println(" | Flex: " + String(flexValue));

  delay(20);
}