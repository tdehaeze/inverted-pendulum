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
constexpr float DEG_PER_ENC_STEP  = 360.0f  / ENC_STEPS_PER_REV;
constexpr float RAD_PER_ENC_STEP  = 6.28318530f / ENC_STEPS_PER_REV;

// 200 steps/rev, 32 microsteps, 16-tooth pulley, 2 mm pitch
constexpr float CART_MM_PER_MICROSTEP = 5.0f / 1000.0f;   // 5 µm/microstep

// ---- Cart limits ----
constexpr float CART_HARD_MM = 200.0f;   // hard stop  (disable motor)
constexpr float CART_SOFT_MM = 160.0f;   // soft limit (block accel toward boundary)

// ---- Velocity / acceleration limits ----
constexpr float ACCEL_MAX_MM_S2 = 10000.0f;   // 10 m/s²
constexpr float VEL_MAX_MM_S    =   300.0f;   // 0.3 m/s

// ---- Angular velocity filter ----
// First-order low-pass on the finite-difference velocity estimate.
// Implements H(s) = s/(s + wc)  with wc = 2*pi*50 rad/s, Dt = 1 ms.
// alpha = exp(-2*pi*50*0.001) ≈ 0.7304
constexpr float ALPHA_VEL = 0.7304f;

// ---- Timing ----
constexpr uint32_t CONTROL_PERIOD_US = 1000;    // 1 kHz
constexpr float    DT                = 1e-3f;   // s

/* =========================================================
   ===================== STEPPER ===========================
   ========================================================= */

IntervalTimer stepTimer;
volatile bool    stepState = false;
volatile int32_t stepCount = 0;

void stepISR() {
    stepState = !stepState;
    digitalWriteFast(STEP_PIN, stepState);
    if (stepState) {
        stepCount += (digitalReadFast(DIR_PIN) ? 1 : -1);
    }
}

void setSpeedSteps(float sps) {
    if (sps <= 0.0f) {
        stepTimer.end();
        return;
    }
    float period_us = 5.0e5f / sps;   // 1e6 / (2 * sps)
    stepTimer.end();
    stepTimer.begin(stepISR, (uint32_t)period_us);
}

/* =========================================================
   ===================== SYSTEM STATE ======================
   ========================================================= */

volatile bool  motor_enabled   = false;
volatile bool  feedback_closed = false;

volatile float openloop_vel_mm_s = 0.0f;
volatile float cart_vel_mm_s     = 0.0f;   // velocity integrated in closed loop

volatile int32_t motor_zero_offset = 0;
volatile int32_t enc_zero_offset   = 0;

/* =========================================================
   ===================== CONTROLLER ========================
   ========================================================= */

enum CtrlMode : uint8_t { MODE_P = 0, MODE_PD = 1, MODE_LQR = 2 };

volatile CtrlMode ctrl_mode     = MODE_PD;
volatile float    Kp            = 0.0f;
volatile float    Kd            = 0.0f;
volatile float    K_lqr[4]      = {0.0f, 0.0f, 0.0f, 0.0f};  // [x, x_dot, phi, phi_dot]
volatile float    theta_ref_deg = 0.0f;   // pendulum angle setpoint [deg]
volatile float    x_ref_mm      = 0.0f;   // cart position setpoint  [mm]

/* =========================================================
   ===================== TELEMETRY =========================
   ========================================================= */

// Packet: header(2) t_us(4) angle_deg(4) pend_vel_deg_s(4)
//         cart_mm(4) cart_vel_mm_s(4) accel_mm_s2(4) feedback_on(1) = 27 bytes
struct __attribute__((packed)) Sample {
    uint16_t header;             // 0xAA55
    uint32_t t_us;
    float    pendulum_deg;
    float    pendulum_vel_deg_s;
    float    cart_mm;
    float    cart_vel_mm_s;
    float    accel_cmd_mm_s2;
    uint8_t  feedback_on;
};

constexpr uint16_t BUFFER_SIZE = 512;
volatile Sample   buffer[BUFFER_SIZE];
volatile uint16_t writeIdx = 0;
volatile uint16_t readIdx  = 0;

/* =========================================================
   ===================== UTILITIES =========================
   ========================================================= */

inline float getPendulumDeg() {
    return (enc.read() - enc_zero_offset) * DEG_PER_ENC_STEP;
}

inline float getCartMm() {
    return (stepCount - motor_zero_offset) * CART_MM_PER_MICROSTEP;
}

inline void motorOff() {
    setSpeedSteps(0);
    cart_vel_mm_s = 0.0f;
    motor_enabled = false;
}

/* =========================================================
   ===================== 1 kHz CONTROL LOOP ================
   ========================================================= */

IntervalTimer controlTimer;
static float prev_pend_deg   = 0.0f;
static float pend_vel_filt   = 0.0f;   // filtered angular velocity [deg/s]

void controlISR() {
    float pend_deg = getPendulumDeg();
    float cart_mm  = getCartMm();

    // Angular velocity: finite difference + first-order low-pass (fc = 50 Hz)
    float pend_vel_raw = (pend_deg - prev_pend_deg) / DT;
    pend_vel_filt = ALPHA_VEL * pend_vel_filt + (1.0f - ALPHA_VEL) * pend_vel_raw;
    float pend_vel_deg_s = pend_vel_filt;
    prev_pend_deg = pend_deg;

    // ---- Hard safety limits ----
    // Stop the cart and disengage feedback, but keep motor enabled so the
    // user (or "Go to Zero") can drive back toward centre.
    bool at_pos_limit = (cart_mm >= CART_HARD_MM);
    bool at_neg_limit = (cart_mm <= -CART_HARD_MM);
    if (at_pos_limit || at_neg_limit) {
        cart_vel_mm_s = 0.0f;
        feedback_closed = false;
    }

    float accel_cmd_mm_s2 = 0.0f;
    float vel_cmd_mm_s    = 0.0f;

    if (motor_enabled) {
        if (feedback_closed) {

            // Convert to SI units for the controller
            float phi_rad     = (pend_deg     - theta_ref_deg) * (3.14159265f / 180.0f);
            float phi_dot_rad = pend_vel_deg_s                 * (3.14159265f / 180.0f);
            float x_m         = (cart_mm      - x_ref_mm)     / 1000.0f;
            float x_dot_m     = cart_vel_mm_s                  / 1000.0f;

            // Controller output [m/s²]
            float accel_m_s2 = 0.0f;
            switch (ctrl_mode) {
                case MODE_P:
                    accel_m_s2 = -Kp * phi_rad;
                    break;
                case MODE_PD:
                    accel_m_s2 = -Kp * phi_rad - Kd * phi_dot_rad;
                    break;
                case MODE_LQR:
                    accel_m_s2 = -(K_lqr[0] * x_m     + K_lqr[1] * x_dot_m
                                 + K_lqr[2] * phi_rad  + K_lqr[3] * phi_dot_rad);
                    break;
            }

            // Convert to mm/s² and clamp
            accel_cmd_mm_s2 = accel_m_s2 * 1000.0f;
            if (accel_cmd_mm_s2 >  ACCEL_MAX_MM_S2) accel_cmd_mm_s2 =  ACCEL_MAX_MM_S2;
            if (accel_cmd_mm_s2 < -ACCEL_MAX_MM_S2) accel_cmd_mm_s2 = -ACCEL_MAX_MM_S2;

            // Soft braking: block acceleration toward the nearest soft limit
            if (cart_mm >  CART_SOFT_MM && accel_cmd_mm_s2 > 0.0f) accel_cmd_mm_s2 = 0.0f;
            if (cart_mm < -CART_SOFT_MM && accel_cmd_mm_s2 < 0.0f) accel_cmd_mm_s2 = 0.0f;

            // Integrate acceleration → velocity
            cart_vel_mm_s += accel_cmd_mm_s2 * DT;

            // Clamp velocity
            if (cart_vel_mm_s >  VEL_MAX_MM_S) cart_vel_mm_s =  VEL_MAX_MM_S;
            if (cart_vel_mm_s < -VEL_MAX_MM_S) cart_vel_mm_s = -VEL_MAX_MM_S;

            vel_cmd_mm_s = cart_vel_mm_s;

        } else {
            // Open loop: direct velocity command
            cart_vel_mm_s = openloop_vel_mm_s;
            vel_cmd_mm_s  = openloop_vel_mm_s;
        }
    }

    // ---- Hard limit: block movement further into the wall ----
    if (at_pos_limit && vel_cmd_mm_s > 0.0f) { vel_cmd_mm_s = 0.0f; cart_vel_mm_s = 0.0f; }
    if (at_neg_limit && vel_cmd_mm_s < 0.0f) { vel_cmd_mm_s = 0.0f; cart_vel_mm_s = 0.0f; }

    // ---- Velocity → stepper speed ----
    float steps_s = vel_cmd_mm_s / CART_MM_PER_MICROSTEP;
    if (steps_s >= 0.0f) {
        digitalWriteFast(DIR_PIN, HIGH);
        setSpeedSteps(steps_s);
    } else {
        digitalWriteFast(DIR_PIN, LOW);
        setSpeedSteps(-steps_s);
    }

    // ---- Ring buffer ----
    uint16_t next = (writeIdx + 1) % BUFFER_SIZE;
    if (next != readIdx) {
        Sample s = {0xAA55, micros(), pend_deg, pend_vel_deg_s,
                    cart_mm, vel_cmd_mm_s, accel_cmd_mm_s2, (uint8_t)feedback_closed};
        memcpy((void*)&buffer[writeIdx], &s, sizeof(Sample));
        writeIdx = next;
    }
}

/* =========================================================
   ===================== SERIAL CMD ========================
   ========================================================= */

void handleCommand(char *cmd) {

    // Motor
    if (!strcmp(cmd, "MOTOR ON"))  { motor_enabled = true; cart_vel_mm_s = 0.0f; return; }
    if (!strcmp(cmd, "MOTOR OFF")) { motorOff();                                  return; }

    // Feedback
    if (!strcmp(cmd, "FB ON"))  { cart_vel_mm_s = 0.0f; feedback_closed = true;  return; }
    if (!strcmp(cmd, "FB OFF")) { cart_vel_mm_s = 0.0f; feedback_closed = false; return; }

    // Zeroing
    if (!strcmp(cmd, "ZERO CART")) { motor_zero_offset = stepCount;   return; }
    if (!strcmp(cmd, "ZERO ENC"))  { enc_zero_offset   = enc.read();  return; }

    // Open-loop velocity [mm/s]
    if (!strncmp(cmd, "VEL ", 4)) { openloop_vel_mm_s = atof(cmd + 4); return; }

    // Controller mode
    if (!strcmp(cmd, "MODE P"))   { ctrl_mode = MODE_P;   return; }
    if (!strcmp(cmd, "MODE PD"))  { ctrl_mode = MODE_PD;  return; }
    if (!strcmp(cmd, "MODE LQR")) { ctrl_mode = MODE_LQR; return; }

    // PD gains
    if (!strncmp(cmd, "SET KP ", 7)) { Kp = atof(cmd + 7); return; }
    if (!strncmp(cmd, "SET KD ", 7)) { Kd = atof(cmd + 7); return; }

    // LQR gains: "SET K k0 k1 k2 k3"
    if (!strncmp(cmd, "SET K ", 6)) {
        float k0, k1, k2, k3;
        if (sscanf(cmd + 6, "%f %f %f %f", &k0, &k1, &k2, &k3) == 4) {
            K_lqr[0] = k0;  K_lqr[1] = k1;
            K_lqr[2] = k2;  K_lqr[3] = k3;
        }
        return;
    }

    // Setpoints
    if (!strncmp(cmd, "SET THETA ", 10)) { theta_ref_deg = atof(cmd + 10); return; }
    if (!strncmp(cmd, "SET X ",      6)) { x_ref_mm      = atof(cmd + 6);  return; }
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
    static char    cmd[64];
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
           Serial.availableForWrite() >= (int)sizeof(Sample)) {
        Serial.write((uint8_t*)&buffer[readIdx], sizeof(Sample));
        readIdx = (readIdx + 1) % BUFFER_SIZE;
    }
}
