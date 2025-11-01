/*
 * MPU6050 Custom Two-Key Controller (W, Left Ctrl)
 * Version: 4.0 (Function-specific)
 * Description: This version controls only two keys: 'W' and 'Left Ctrl'.
 * Each key has its own independent activation threshold.
 * The code is structured to handle simultaneous key presses.
 *
 * Code Modified by: Code Companion
 * Current Time: Saturday, October 11, 2025 at 8:01:14 AM CST
 * Location: Taiwan
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

// --- Tweakable Parameters ---
// Independent tilt thresholds for each key (in degrees)
const int W_KEY_THRESHOLD = 10;    // Threshold for the 'W' key
const int CTRL_KEY_THRESHOLD = 30; // Threshold for the 'Left Ctrl' key

// --- Key State Tracking ---
// A struct to independently track the state of each key
struct KeyStates {
  bool w = false;
  bool left_ctrl = false;
};
KeyStates keyStates;

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("MPU6050 chip not found");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  Serial.println("Controller ready (W and Left Ctrl mode).");

  // Initialize Keyboard functionality
  Keyboard.begin();
}

// Helper function to manage key presses and releases efficiently
// It only sends a command when the key's state changes.
// Note: The key type is uint8_t to handle special keys like KEY_LEFT_CTRL
void handleKeyPress(uint8_t key, bool shouldBePressed, bool &isPressed) {
  if (shouldBePressed && !isPressed) {
    Keyboard.press(key);
    isPressed = true;
  } else if (!shouldBePressed && isPressed) {
    Keyboard.release(key);
    isPressed = false;
  }
}

void loop() {
  // Get new sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Calculate Tilt Angles ---
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / M_PI;
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  
  // --- Independent Control Logic for Each Key ---
  handleKeyPress('w', roll < -W_KEY_THRESHOLD, keyStates.w);
  handleKeyPress(KEY_LEFT_CTRL, roll < -CTRL_KEY_THRESHOLD, keyStates.left_ctrl);

  // --- Update Serial Monitor Output ---
  String pressedKeysStr = "";
  if (keyStates.w) {
    // Corrected the status message to show the 'roll' value that triggers 'w'
    pressedKeysStr += "W(Roll:" + String(roll) + ") ";
  }
  if (keyStates.left_ctrl) {
    pressedKeysStr += "L_CTRL(Roll:" + String(roll) + ") ";
  }

  if (pressedKeysStr.length() == 0) {
    Serial.println("Status: IDLE");
  } else {
    Serial.println("Keys: " + pressedKeysStr);
  }

  delay(50);
}