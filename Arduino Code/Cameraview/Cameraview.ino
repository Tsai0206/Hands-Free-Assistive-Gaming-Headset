/*
 * Code to view Gyroscope values and corresponding mouse output on the Serial Monitor.
 * This version is CORRECTED to properly scale sensor values for mouse movement.
 * It uses a multiplier and includes a deadzone for stability.
 *
 * Original concept by Gabry295, corrected by Code Companion.
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Mouse.h>

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

// --- Tweakable Parameters ---
// MOUSE MULTIPLIER: A LARGER value makes the mouse move faster (more sensitive).
// We now MULTIPLY the sensor reading by this value. Start with a value around 5.0.
float mouseMultiplier = 20.0; 

// DEADZONE: Sensor readings within this range will be ignored to prevent jitter.
// If the mouse drifts when still, increase this value slightly.
float deadzoneThreshold = 0.1;

// --- Global Variables for Calibration ---
float gyroX_offset = 0.0;
float gyroZ_offset = 0.0;

void setup() {
  Serial.begin(9600);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // --- Automatic Gyroscope Calibration ---
  Serial.println("Calibrating Gyro... Do not move the sensor.");
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
  Serial.print("Gyro X offset: ");
  Serial.println(gyroX_offset);
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZ_offset);
  
  // Initialize the mouse
  Mouse.begin();
}

void loop() {
  // Get new sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration offset to get the true rotation speed.
  float calibrated_gx = g.gyro.x - gyroX_offset;
  float calibrated_gz = g.gyro.z - gyroZ_offset;

  // Initialize mouse movement values
  int mouse_vx = 0;
  int mouse_vy = 0;

  // Check if the X-axis rotation is outside the deadzone
  if (abs(calibrated_gz) > deadzoneThreshold) {
    mouse_vx = -(int)(calibrated_gz * mouseMultiplier);
  }

  // Check if the Z-axis rotation is outside the deadzone
  if (abs(calibrated_gx) > deadzoneThreshold) {
    // Original script used -(gz) for vertical movement
    mouse_vy = (int)(calibrated_gx * mouseMultiplier);
  }

  // Print the raw and calculated values to the Serial Monitor for testing.
  Serial.print("gx = ");
  Serial.print(g.gyro.x);
  Serial.print(" | gz = ");
  Serial.print(g.gyro.z);
  
  Serial.print("      | X = ");
  Serial.print(mouse_vx);
  Serial.print(" | Y = ");
  Serial.println(mouse_vy);
  
  // Move the mouse if there is any calculated movement
  if (mouse_vx != 0 || mouse_vy != 0) {
    Mouse.move(mouse_vx, mouse_vy);
  }
  
  delay(20);
}