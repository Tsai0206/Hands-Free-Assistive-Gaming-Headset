/*
 * MPU6050 Hybrid Controller (Keyboard + Clutch Mouse)
 * Version: 5.0 (Integration)
 * Description: This script combines two functions into one controller.
 * 1. Keyboard movement (W, Left Ctrl) based on tilt is ALWAYS ON.
 * 2. Mouse camera view based on gyro is ACTIVATED ONLY when the 
 * FlexiForce sensor (clutch) is pressed.
 *
 * Code Integrated by: Code Companion
 * Current Time: Saturday, October 11, 2025 at 8:16:46 AM CST
 * Location: Taiwan
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Keyboard.h>
#include <Mouse.h>

// Create a single MPU6050 object
Adafruit_MPU6050 mpu;

// --- Tweakable Parameters (All in one place) ---

// -- Keyboard Control --
const int W_KEY_THRESHOLD = 10;    // Tilt threshold for the 'W' key (degrees)
const int CTRL_KEY_THRESHOLD = 40; // Tilt threshold for the 'Left Ctrl' key (degrees)

// -- Mouse "Clutch" Control --
const int flexPin = A1;          // Analog pin for the FlexiForce sensor
int flexThreshold = 850;         // Pressure threshold to activate mouse view
float mouseMultiplierX = 45.0;   // Mouse sensitivity for left/right view
float mouseMultiplierY = 20.0;   // Mouse sensitivity for up/down view
float deadzoneThreshold = 0.1;   // Gyro deadzone to prevent mouse drift

// --- State Tracking ---
struct KeyStates {
  bool w = false;
  bool left_ctrl = false;
};
KeyStates keyStates;

float gyroX_offset = 0.0;
float gyroZ_offset = 0.0;

// Helper function to manage key presses and releases
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

  // Gyro calibration is needed for the mouse function
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
  Serial.println("Calibration Complete.");

  // Initialize both Keyboard and Mouse
  Keyboard.begin();
  Mouse.begin();
  
  Serial.println("Hybrid Controller is ready.");
}

void loop() {
  // --- Step 1: Read sensor data once at the beginning of the loop ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Step 2: ALWAYS-ON LOGIC - Keyboard Movement Control ---
  // This part runs continuously, regardless of the clutch.
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / M_PI;
  handleKeyPress('w', roll < -W_KEY_THRESHOLD, keyStates.w);
  handleKeyPress(KEY_LEFT_CTRL, roll < -CTRL_KEY_THRESHOLD, keyStates.left_ctrl); // Assuming Left Tilt for Ctrl

  // --- Step 3: CLUTCH-ACTIVATED LOGIC - Mouse Camera Control ---
  // This part only runs when the FlexiForce sensor is pressed.
  int flexValue = analogRead(flexPin);
  bool mouseModeIsOn = (flexValue > flexThreshold);

  if (mouseModeIsOn) {
    // Calculate calibrated gyro speeds
    float calibrated_gx = g.gyro.x - gyroX_offset;
    float calibrated_gz = g.gyro.z - gyroZ_offset;

    int mouse_vx = 0;
    int mouse_vy = 0;

    // Map gyro rotation to mouse movement
    if (abs(calibrated_gz) > deadzoneThreshold) {
      mouse_vx = -(int)(calibrated_gz * mouseMultiplierX);
    }
    if (abs(calibrated_gx) > deadzoneThreshold) {
      mouse_vy = (int)(calibrated_gx * mouseMultiplierY);
    }

    // Move the mouse
    if (mouse_vx != 0 || mouse_vy != 0) {
      Mouse.move(mouse_vx, mouse_vy);
    }
  }

  // --- Step 4: Unified Serial Monitor Output ---
  // Build the status string for keys
  String pressedKeysStr = "";
  if (keyStates.w) pressedKeysStr += "W ";
  if (keyStates.left_ctrl) pressedKeysStr += "L_CTRL ";
  if (pressedKeysStr.length() == 0) pressedKeysStr = "None";

  // Print the combined status
  Serial.print("Keys: " + pressedKeysStr);
  Serial.print(" | Mouse Mode: ");
  if (mouseModeIsOn) {
    Serial.println("ON (Flex: " + String(flexValue) + ")");
  } else {
    Serial.println("OFF (Flex: " + String(flexValue) + ")");
  }

  delay(20);
}