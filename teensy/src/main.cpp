#include <Arduino.h>
#include <IntervalTimer.h>
#include <Encoder.h>

/* =========================================================
   ===================== CONSTANTS  ========================
   ========================================================= */

// ---- Stepper pins ----
const uint8_t STEP_PIN = 10;
const uint8_t DIR_PIN  = 11;

// ---- Encoder ----
Encoder enc(2, 3);

// ---- Conversions ----
constexpr float ENC_STEPS_PER_REV = 4096.0f;
constexpr float DEG_PER_STEP = 360.0f / ENC_STEPS_PER_REV;

// 200 steps per turn
// 32 micro-steps
// 16 theeth, 2mm pitch
constexpr float CART_UM_PER_MICROSTEP = 5.0f;
constexpr float CART_MM_PER_MICROSTEP = CART_UM_PER_MICROSTEP / 1000.0f;

// ---- Cart limits ----
constexpr float CART_MIN_MM = -200.0f;
constexpr float CART_MAX_MM =  200.0f;

// ---- Timing ----
constexpr uint32_t CONTROL_PERIOD_US = 1000; // 1 kHz

/* =========================================================
   ===================== STEPPER ===========================
   ========================================================= */

IntervalTimer stepTimer;
volatile bool stepState = false;
volatile int32_t stepCount = 0;

// Function used to generate the steps to move the motor
void stepISR() {
    stepState = !stepState;
    digitalWriteFast(STEP_PIN, stepState);
    if (stepState) {
        stepCount += (digitalReadFast(DIR_PIN) ? 1 : -1);
    }
}

// Configure the motor velocity => Use a timer
void setSpeedSteps(float sps) {
    // Velocity should always be positive
    // Otherwise, disable the motor
    if (sps <= 0.0f) {
        stepTimer.end();
        return;
    }

    float freq = 2.0f * sps;
    float period_us = 1e6f / freq;

    stepTimer.end();
    stepTimer.begin(stepISR, (uint32_t)period_us);
}

/* =========================================================
   ===================== ETAT SYSTEME ======================
   ========================================================= */

volatile bool motor_enabled   = false;
volatile bool feedback_closed = false;

volatile float openloop_vel_mm_s = 0.0f;

volatile int32_t motor_zero_offset = 0;
volatile int32_t enc_zero_offset   = 0;

volatile bool indexDetected = false;

/* =========================================================
   ===================== TELEMETRIE ========================
   ========================================================= */

struct __attribute__((packed)) Sample {
    uint16_t header;          // 0xAA55
    uint32_t t_us;            // Time in us
    float pendulum_deg;       //
    float cart_mm;            //
    float cart_vel_mm_s;      //
    uint8_t feedback_on;      //
};

constexpr uint16_t BUFFER_SIZE = 512;
volatile Sample buffer[BUFFER_SIZE];
volatile uint16_t writeIdx = 0;
volatile uint16_t readIdx  = 0;

/* =========================================================
   ===================== UTILITAIRES =======================
   ========================================================= */

inline float getPendulumDeg() {
    long raw = enc.read() - enc_zero_offset;
    return raw * DEG_PER_STEP;
}

inline float getCartMm() {
    long steps = stepCount - motor_zero_offset;
    return steps * CART_MM_PER_MICROSTEP;
}

inline void motorOff() {
    setSpeedSteps(0);
    motor_enabled = false;
}

/* =========================================================
   ===================== BOUCLE 1 kHz ======================
   ========================================================= */

IntervalTimer controlTimer;

void controlISR() {
    float pendulum_deg = getPendulumDeg();
    float cart_mm      = getCartMm();

    // ---- Sécurité limites ----
    if (cart_mm < CART_MIN_MM || cart_mm > CART_MAX_MM) {
        motorOff();
        feedback_closed = false;
    }

    float vel_cmd_mm_s = 0.0f;

    if (motor_enabled) {
        if (feedback_closed) {
            // ===== CLOSED LOOP (PID / LQR à venir) =====
            vel_cmd_mm_s = 0.0f; // placeholder
        } else {
            // ===== OPEN LOOP =====
            vel_cmd_mm_s = openloop_vel_mm_s;
        }
    }

    // ---- Conversion mm/s → steps/s ----
    float steps_s = vel_cmd_mm_s / CART_MM_PER_MICROSTEP;

    if (steps_s >= 0) {
        digitalWriteFast(DIR_PIN, HIGH);
        setSpeedSteps(steps_s);
    } else {
        digitalWriteFast(DIR_PIN, LOW);
        setSpeedSteps(-steps_s);
    }

    // ---- Ring buffer ----
    uint16_t next = (writeIdx + 1) % BUFFER_SIZE;
    if (next != readIdx) {
        buffer[writeIdx] = {
            0xAA55,
            micros(),
            pendulum_deg,
            cart_mm,
            vel_cmd_mm_s,
            feedback_closed
        };
        writeIdx = next;
    }
}

/* =========================================================
   ===================== SERIAL CMD ========================
   ========================================================= */

void handleCommand(char *cmd) {
    if (!strcmp(cmd, "MOTOR ON")) motor_enabled = true;
    if (!strcmp(cmd, "MOTOR OFF")) motorOff();

    if (!strcmp(cmd, "FB ON")) feedback_closed = true;
    if (!strcmp(cmd, "FB OFF")) feedback_closed = false;

    if (!strcmp(cmd, "ZERO CART")) motor_zero_offset = stepCount;
    if (!strcmp(cmd, "ZERO ENC"))  enc_zero_offset   = enc.read();

    if (!strncmp(cmd, "VEL ", 4)) {
        openloop_vel_mm_s = atof(cmd + 4);
    }
}

/* =========================================================
   ===================== SETUP =============================
   ========================================================= */

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    digitalWriteFast(DIR_PIN, HIGH);

    controlTimer.begin(controlISR, CONTROL_PERIOD_US);
}

/* =========================================================
   ===================== LOOP ==============================
   ========================================================= */

void loop() {
    // ---- Command parsing ----
    static char cmd[32];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            cmd[idx] = 0;
            handleCommand(cmd);
            idx = 0;
        } else if (idx < sizeof(cmd) - 1) {
            cmd[idx++] = c;
        }
    }

    // ---- Streaming ----
    while (readIdx != writeIdx &&
           Serial.availableForWrite() >= sizeof(Sample)) {
        Serial.write((uint8_t*)&buffer[readIdx], sizeof(Sample));
        readIdx = (readIdx + 1) % BUFFER_SIZE;
    }
}
