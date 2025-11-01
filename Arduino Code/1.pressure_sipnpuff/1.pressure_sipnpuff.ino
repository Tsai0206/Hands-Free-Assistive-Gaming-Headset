/*
 * Sip & Puff Controller for Minecraft
 * Version 2.5 (Combined Logic: Hold-to-Jump + Hold-to-Click)
 * Description: This version combines the user's requests.
 * - Soft Puff: Holds Left Click (after an evaluation period).
 * - Hard Puff: Holds Space Bar (Jump).
 * - These two actions are mutually exclusive.
 */

#include <Mouse.h>
#include <Keyboard.h>

// --- Tweakable Parameters ---
const int SENSOR_PIN = A0;
const int HARD_PUFF_THRESHOLD = 30;
const int SOFT_PUFF_THRESHOLD = 7;
const int SIP_THRESHOLD = -5;
const int NEUTRAL_DEADZONE = 3;
const int EVALUATION_PERIOD = 5; // 10 measurements
// ------------------------------------------------

// --- Global Variables ---
int baselinePressure = 0;
bool discreteActionTaken = false; // This is now ONLY for Sip (Right Click)
bool isLeftClickHeld = false;
bool isJumpHeld = false; // --- NEW: State variable for holding Jump ---
bool isEvaluatingPuff = false;
int puffMeasurementCounter = 0;

void setup() {
  Serial.begin(115200);
  Mouse.begin();
  Keyboard.begin();

  // --- Automatic Baseline Calibration ---
  Serial.println("Calibrating pressure sensor... Do not sip or puff.");
  long pressureSum = 0;
  for (int i = 0; i < 500; i++) {
    pressureSum += analogRead(SENSOR_PIN);
    delay(2);
  }
  baselinePressure = pressureSum / 500;
  
  Serial.print("Calibration Complete. Baseline pressure: ");
  Serial.println(baselinePressure);
  Serial.println("Controller is ready. Open Minecraft!");
}

void loop() {
  int currentPressure = analogRead(SENSOR_PIN);
  int pressureDelta = currentPressure - baselinePressure;

  // Define all possible states
  bool isNeutral = abs(pressureDelta) < NEUTRAL_DEADZONE;
  bool isHardPuff = pressureDelta > HARD_PUFF_THRESHOLD;
  bool isSip = pressureDelta < SIP_THRESHOLD;
  bool isSoftPuffRange = (pressureDelta > SOFT_PUFF_THRESHOLD) && (pressureDelta <= HARD_PUFF_THRESHOLD);

  // --- REVISED: Main Control Logic ---

  // --- Part 1: Handle Neutral State (Reset Everything) ---
  if (isNeutral) {
    if (discreteActionTaken) {
      discreteActionTaken = false;
      Serial.println("Action: RESET (Returned to neutral)");
    }
    if (isLeftClickHeld) {
      Mouse.release(MOUSE_LEFT);
      isLeftClickHeld = false;
      Serial.println("Action: LEFT CLICK (Hold END)");
    }
    if (isJumpHeld) { // --- NEW: Release jump key on neutral ---
      Keyboard.release(' ');
      isJumpHeld = false;
      Serial.println("Action: JUMP (Hold END)");
    }
    isEvaluatingPuff = false;
    puffMeasurementCounter = 0;
  }

  // --- Part 2: Handle Sip (Discrete Action) ---
  else if (isSip && !discreteActionTaken) {
    Serial.print("Action: RIGHT CLICK (Sip). Delta: ");
    Serial.println(pressureDelta);
    Mouse.click(MOUSE_RIGHT);
    discreteActionTaken = true; // Lock Sip until neutral

    // Stop all other actions
    isEvaluatingPuff = false;
    if (isLeftClickHeld) { Mouse.release(MOUSE_LEFT); isLeftClickHeld = false; }
    if (isJumpHeld) { Keyboard.release(' '); isJumpHeld = false; }
  }
  
  // --- Part 3: Handle Puffing (Continuous Actions) ---
  // We only handle puffs if we are not in the "Sip" lock.
  else if (!discreteActionTaken) {
    
    // Priority 1: Hard Puff (Hold Jump)
    if (isHardPuff) {
      // If we aren't already holding Jump, press it.
      if (!isJumpHeld) {
        Serial.print("Action: JUMP (Hold START). Delta: ");
        Serial.println(pressureDelta);
        Keyboard.press(' ');
        isJumpHeld = true;
      }

      // *** FIX ***
      // We are hard-puffing, so we MUST cancel any soft-puff logic.
      isEvaluatingPuff = false;
      puffMeasurementCounter = 0;
      // And we MUST release the left click if it was held.
      if (isLeftClickHeld) {
        Mouse.release(MOUSE_LEFT);
        isLeftClickHeld = false;
        Serial.println("Action: (Auto-Release Left Click)");
      }
    }
    
    // Priority 2: Soft Puff (Hold Left Click)
    else if (isSoftPuffRange) {
      // We are in soft range, so we MUST release the jump key.
      if (isJumpHeld) {
        Keyboard.release(' ');
        isJumpHeld = false;
        Serial.println("Action: JUMP (Hold END)");
      }
      
      // Now, run the evaluation logic for Left Click.
      if (isLeftClickHeld) {
        // (do nothing, just keep holding)
      } 
      else if (!isEvaluatingPuff) {
        isEvaluatingPuff = true;
        puffMeasurementCounter = 0;
        Serial.println("Action: (Evaluating Puff...)");
      }
      else { // isEvaluatingPuff is true
        puffMeasurementCounter++;
        Serial.print("Action: (Evaluating... ");
        Serial.print(puffMeasurementCounter);
        Serial.println(")");
        
        if (puffMeasurementCounter >= EVALUATION_PERIOD) {
          Serial.print("Action: LEFT CLICK (Hold START). Delta: ");
          Serial.println(pressureDelta);
          Mouse.press(MOUSE_LEFT);
          isLeftClickHeld = true;
          isEvaluatingPuff = false;
        }
      }
    } 
    
    // Priority 3: Not in any "action" zone (but not neutral)
    else {
      // Release both holds and cancel evaluation
      if (isLeftClickHeld) {
        Mouse.release(MOUSE_LEFT);
        isLeftClickHeld = false;
        Serial.println("Action: LEFT CLICK (Hold END)");
      }
      if (isJumpHeld) {
        Keyboard.release(' ');
        isJumpHeld = false;
        Serial.println("Action: JUMP (Hold END)");
      }
      if (isEvaluatingPuff) {
        isEvaluatingPuff = false;
        puffMeasurementCounter = 0;
        Serial.println("Action: (Evaluation Canceled)");
      }
    }
  }

  // --- Debug output ---
  if (!discreteActionTaken && !isLeftClickHeld && !isJumpHeld && !isEvaluatingPuff && !isNeutral) {
    Serial.print("Waiting for action... Pressure Delta: ");
    Serial.println(pressureDelta);
  }

  delay(20);
}