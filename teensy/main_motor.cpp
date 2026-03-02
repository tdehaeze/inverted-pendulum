#include <Arduino.h>
#include <IntervalTimer.h>

// Pin mapping (modifie à ton goût)
const uint8_t STEP_PIN = 10;
const uint8_t DIR_PIN  = 11;

IntervalTimer stepTimer;

// state for toggling
volatile bool stepState = false;
volatile uint32_t stepCount = 0;

// Desired step frequency (steps per second)
volatile float steps_per_sec = 500.0f; // exemple
// Timer frequency = 2 * steps_per_sec
volatile float timerFreq = steps_per_sec * 2.0f;

// ISR called at timerFreq Hz: toggles STEP pin
void onStepTimer() {
  stepState = !stepState;
  digitalWriteFast(STEP_PIN, stepState ? HIGH : LOW);

  // Count a full step on rising edge (when stepState == true)
  if (stepState) {
    stepCount++;
  }
}

void setStepsPerSec(float sps) {
  if (sps < 0.1f) sps = 0.1f;
  steps_per_sec = sps;
  timerFreq = steps_per_sec * 2.0f;
  // reconfigure timer: stop, begin with new period (in microseconds)
  stepTimer.end();
  // period in microseconds = 1e6 / timerFreq
  float period_us = 1e6f / timerFreq;
  stepTimer.begin(onStepTimer, (uint32_t)period_us);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("DRV8825 simple test (microstep x32)");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // default direction
  digitalWrite(DIR_PIN, HIGH);

  // start timer
  setStepsPerSec(steps_per_sec);

  Serial.print("Initial steps/s = ");
  Serial.println(steps_per_sec);
}

unsigned long lastPrint = 0;
void loop() {
  // simple serial commands to control:
  // "SPS=1000" -> set steps per second
  // "DIR=0" or "DIR=1" -> set direction
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("SPS=")) {
      float v = cmd.substring(4).toFloat();
      setStepsPerSec(v);
      Serial.print("Set steps_per_sec = ");
      Serial.println(v);
    } else if (cmd.startsWith("DIR=")) {
      int d = cmd.substring(4).toInt();
      digitalWrite(DIR_PIN, d ? HIGH : LOW);
      Serial.print("Set DIR = ");
      Serial.println(d);
    } else if (cmd == "STOP") {
      stepTimer.end();
      Serial.println("Timer stopped -> motor idle.");
    } else if (cmd == "START") {
      setStepsPerSec(steps_per_sec);
      Serial.println("Timer started.");
    } else if (cmd == "COUNT") {
      Serial.print("Step count = ");
      Serial.println(stepCount);
    } else {
      Serial.println("Unknown command. Use SPS=..., DIR=0/1, START/STOP, COUNT");
    }
  }

  // periodic status output
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("SPS=");
    Serial.print(steps_per_sec);
    Serial.print("  DIR=");
    Serial.print(digitalRead(DIR_PIN));
    Serial.print("  Count=");
    Serial.println(stepCount);
  }
}

// #include <Arduino.h>
// #include <IntervalTimer.h>

// // ==========================
// // PID Parameters (modifiable via USB)
// // ==========================
// volatile float Kp = 5.0f;
// volatile float Ki = 0.0f;
// volatile float Kd = 0.0f;
// volatile float setpoint = 0.0f;

// // ==========================
// // Simulated system
// // ==========================
// // Simulated encoder position (rad)
// volatile float position = 0.0f;
// // Simulated velocity (rad/s)
// volatile float velocity = 0.0f;

// // PID internal
// volatile float integral = 0.0f;
// volatile float lastError = 0.0f;

// // Timer for 1 kHz loop
// IntervalTimer pidTimer;

// // Limit loop frequency of streaming
// unsigned long lastStream = 0;

// // ============================================
// // 1 kHz CONTROL LOOP
// // ============================================
// void pidISR() {
//     float dt = 0.001f;  // 1 ms

//     // ---- Simulated system dynamics ----
//     // Very basic first-order model:
//     // velocity += (command - damping*velocity) * dt
//     // position += velocity * dt

//     // Here command = PID output
//     // (We'll compute PID output below)

//     // ---- PID ----
//     float error = setpoint - position;

//     integral += error * dt;
//     float derivative = (error - lastError) / dt;

//     float command = Kp * error + Ki * integral + Kd * derivative;
//     lastError = error;

//     // ---- UPDATE SIMULATED SYSTEM ----
//     float damping = 0.2f;     // Make the system stable-ish
//     velocity += (command - damping * velocity) * dt;
//     position += velocity * dt;

//     // That's it. A tiny physics engine :)
// }

// // ============================================
// // Parse serial commands
// // Ex: "P=12.3", "I=0.1", "D=2.0", "S=1.57"
// // ============================================
// void parseCommand(String cmd) {
//     cmd.trim();
//     if (cmd.startsWith("P=")) {
//         Kp = cmd.substring(2).toFloat();
//     } else if (cmd.startsWith("I=")) {
//         Ki = cmd.substring(2).toFloat();
//     } else if (cmd.startsWith("D=")) {
//         Kd = cmd.substring(2).toFloat();
//     } else if (cmd.startsWith("S=")) {
//         setpoint = cmd.substring(2).toFloat();
//     }

//     Serial.print("Updated -> Kp=");
//     Serial.print(Kp);
//     Serial.print(" Ki=");
//     Serial.print(Ki);
//     Serial.print(" Kd=");
//     Serial.print(Kd);
//     Serial.print(" Setpoint=");
//     Serial.println(setpoint);
// }

// // ============================================
// // SETUP
// // ============================================
// void setup() {
//     Serial.begin(115200);
//     delay(1000);

//     Serial.println("Starting simulation...");

//     pidTimer.begin(pidISR, 1000);  // 1 kHz
// }

// // ============================================
// // MAIN LOOP
// // ============================================
// void loop() {
//     // Handle incoming USB commands
//     if (Serial.available()) {
//         String cmd = Serial.readStringUntil('\n');
//         parseCommand(cmd);
//     }

//     // Stream data @ 50 Hz
//     if (millis() - lastStream > 20) {
//         lastStream = millis();
//         Serial.print("POS=");
//         Serial.print(position, 6);
//         Serial.print(" VEL=");
//         Serial.print(velocity, 6);
//         Serial.print(" SP=");
//         Serial.println(setpoint, 3);
//     }
// }
