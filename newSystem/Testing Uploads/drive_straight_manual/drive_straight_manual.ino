/*
 * drive_straight_manual.ino — Manual straight-line tuning tool
 *
 * No camera, no UDP, no PID — just two PWM values you tweak by hand until
 * the robot tracks straight. Once it does, note the ratio between
 * LEFT_PWM and RIGHT_PWM; that's your motor mismatch correction factor.
 *
 * HOW TO USE:
 *   1. Flash this sketch.
 *   2. Place the robot at one end of a straight line (tape on the floor).
 *   3. Open Serial Monitor (115200 baud) and reset the board, or press
 *      the button on GPIO0 (BOOT) to trigger a run — see RUN_PIN below.
 *   4. Watch which way it drifts.
 *   5. If it drifts LEFT, increase LEFT_PWM slightly (or decrease RIGHT_PWM).
 *      If it drifts RIGHT, increase RIGHT_PWM slightly (or decrease LEFT_PWM).
 *   6. Re-flash (or just edit and re-upload) and test again.
 *   7. Repeat until it tracks straight over ~1m.
 *
 * Once straight, the ratio RIGHT_PWM / LEFT_PWM is your per-robot
 * trim factor — apply it the same way in your real drive code.
 */

#include <Arduino.h>

// ── Motor driver pins (from spec sheet) ──────────────────────────────────────
#define PWMA   4
#define AIN2   5
#define AIN1   6
#define STBY   7
#define BIN1   8
#define BIN2   9
#define PWMB  10

// ── PWM config ────────────────────────────────────────────────────────────────
#define PWM_FREQ        20000
#define PWM_RESOLUTION      8
#define PWM_MAX_SAFE      179     // 70% cap — do not exceed

// ── TUNE THESE TWO VALUES ─────────────────────────────────────────────────────
// Start with both equal, then adjust based on which way it drifts.
int LEFT_PWM  = 130;
int RIGHT_PWM = 120;

// ── Run parameters ────────────────────────────────────────────────────────────
#define RUN_DURATION_MS   3000     // how long to drive forward
#define RUN_PIN              0     // BOOT button on most ESP32-S3 boards


void motors_init() {
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);

    pinMode(RUN_PIN, INPUT_PULLUP);   // BOOT button: LOW when pressed
}

int clamp_pwm(int pwm) {
    if (pwm < 0)            pwm = 0;
    if (pwm > PWM_MAX_SAFE) pwm = PWM_MAX_SAFE;
    return pwm;
}

void drive_forward(int left_pwm, int right_pwm) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    ledcWrite(PWMA, clamp_pwm(left_pwm));
    ledcWrite(PWMB, clamp_pwm(right_pwm));
}

void motors_stop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMA, 0);
    ledcWrite(PWMB, 0);
}

void run_once() {
    Serial.printf("\n[Run] LEFT_PWM=%d  RIGHT_PWM=%d  for %d ms\n",
                  LEFT_PWM, RIGHT_PWM, RUN_DURATION_MS);
    drive_forward(LEFT_PWM, RIGHT_PWM);
    delay(RUN_DURATION_MS);
    motors_stop();
    Serial.println("[Run] Stopped. Observe drift direction, adjust PWM values, re-flash.");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    motors_init();
    motors_stop();
    Serial.println("[Boot] Manual drive-straight tuner ready.");
    Serial.println("Press the BOOT button to run, or it will auto-run once in 3s.");

    delay(3000);   // gives you time to walk away / position the robot
    run_once();
}

void loop() {
    // Press BOOT button to repeat the same run without re-flashing
    if (digitalRead(RUN_PIN) == LOW) {
        delay(50);   // debounce
        if (digitalRead(RUN_PIN) == LOW) {
            run_once();
            while (digitalRead(RUN_PIN) == LOW) delay(10);  // wait for release
        }
    }
    delay(10);
}
