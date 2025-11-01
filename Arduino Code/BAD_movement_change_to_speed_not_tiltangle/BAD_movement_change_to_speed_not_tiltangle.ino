/*
 * MPU6050 Hybrid Controller (Remapped Controls)
 * Version: 7.0 (Acceleration-Based Sprint Latch)
 * Description: This version changes the movement logic. A forward acceleration
 * "latches" the W+Ctrl keys on, and a backward acceleration turns them off.
 * This is based on dynamic movement, not static tilt.
 *
 * Code Modified by: Code Companion
 * Current Time: Friday, October 17, 2025 at 10:05:08 AM CST
 * Location: Taiwan
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

Adafruit_MPU6050 mpu;

// --- Tweakable Parameters (All in one place) ---
// -- Acceleration-Based Keyboard Control --
const float FORWARD_ACCEL_THRESHOLD = 3.0; // Forward acceleration to turn sprint ON.
// --- MODIFIED --- New threshold for the "off" switch
const float SIDEWAYS_SHAKE_THRESHOLD = 4.0; // Sideways acceleration (shake) to turn sprint OFF.

// -- Mouse "Clutch" and Jump Control --
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

bool isSprintingForward = false;
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

  float calibrated_gx = g.gyro.x - gyroX_offset;

  // --- MODIFIED: Acceleration-Based Sprint Latch with Shake-to-Stop ---
  float accel_y = a.acceleration.y;
  float accel_x = a.acceleration.x; // We now need the X-axis acceleration

  // Check for events to toggle the sprint state
  if (!isSprintingForward && accel_y > FORWARD_ACCEL_THRESHOLD) {
    isSprintingForward = true; // A forward jerk turns the sprint ON
  } else if (isSprintingForward && abs(accel_x) > SIDEWAYS_SHAKE_THRESHOLD) {
    isSprintingForward = false; // A sideways jerk turns the sprint OFF
  }

  // Apply the current state to the keys
  handleKeyPress('w', isSprintingForward, keyStates.w);
  handleKeyPress(KEY_LEFT_CTRL, isSprintingForward, keyStates.left_ctrl);
  handleKeyPress('s', false, keyStates.s);

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
    int mouse_vx = 0;
    if (abs(calibrated_gx) > deadzoneThreshold) {
      mouse_vx = (int)(calibrated_gx * mouseMultiplierX);
    }
    if (mouse_vx != 0) {
      Mouse.move(mouse_vx, 0);
    }
  }
  clutchWasEngaged = clutchIsEngaged;

  // --- Unified Serial Monitor Output ---
  String moveState = "IDLE";
  if (isSprintingForward) {
    moveState = "SPRINTING (LOCKED)";
  }
  
  Serial.print("Move State: " + moveState + " (Y:" + String(accel_y, 1) + ", X:" + String(accel_x, 1) + ")");
  Serial.print(" | Mouse: ");
  if (clutchIsEngaged) { Serial.print("ON"); } else { Serial.print("OFF"); }
  Serial.print(" (Rot. X: " + String(calibrated_gx, 2) + ")");
  Serial.print(" | Jump Ready: ");
  if (jumpWasTriggeredInThisPress) { Serial.print("NO"); } else { Serial.print("YES"); }
  Serial.println(" | Flex: " + String(flexValue));

  delay(20);
}