#include <Wire.h>
#include <AS5600.h>

AS5600 as5600;

#define IN3 18
#define IN4 19
#define ENB 5

float zeroOffset  = 0;
float targetAngle = 0;
bool  running     = false;

float Kp = 1.2;
float Ki = 0.0;
float Kd = 0.35;

float error      = 0;
float prevError  = 0;
float integral   = 0;
float derivative = 0;   
int   pwm        = 0;   

unsigned long prevTime = 0;

const int   DEADBAND  = 90;
const int   MAX_PWM   = 95;
const float TOLERANCE = 2.5;
const float SLOW_ZONE = 35;

float shortestError(float target, float current) {
  float e = target - current;
  if (e >  180) e -= 360;
  if (e < -180) e += 360;
  return e;
}

void stopMotor() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  pwm = 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7);

  if (!as5600.begin()) {
    Serial.println("AS5600 not detected");
    while (1);
  }

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotor();
  delay(500);

  int raw    = as5600.readAngle();
  zeroOffset = raw * AS5600_RAW_TO_DEGREES;
  prevTime   = millis();

  Serial.println("=== PID Motor Controller ===");
  Serial.println("  a<deg>  → move to angle");
  Serial.println("  z       → zero here");
  Serial.println("  s       → stop");
  Serial.println("  kp/ki/kd<val> → set gains");
  Serial.println("============================");
}

void loop() {

  // ── Serial Commands ────────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.startsWith("a")) {
      targetAngle = cmd.substring(1).toFloat();
      integral    = 0;
      prevError   = 0;
      running     = true;
      Serial.print("→ Target: "); Serial.print(targetAngle); Serial.println("°");
    }
    else if (cmd == "z") {
      int raw    = as5600.readAngle();
      zeroOffset = raw * AS5600_RAW_TO_DEGREES;
      running    = false;
      stopMotor();
      Serial.println("✓ Zeroed");
    }
    else if (cmd == "s") {
      running = false;
      stopMotor();
      Serial.println("✓ Stopped");
    }
    else if (cmd.startsWith("kp")) { Kp = cmd.substring(2).toFloat(); Serial.print("Kp="); Serial.println(Kp); }
    else if (cmd.startsWith("ki")) { Ki = cmd.substring(2).toFloat(); Serial.print("Ki="); Serial.println(Ki); }
    else if (cmd.startsWith("kd")) { Kd = cmd.substring(2).toFloat(); Serial.print("Kd="); Serial.println(Kd); }
  }

  // ── Read Angle ─────────────────────────────────────────
  int   raw     = as5600.readAngle();
  float degrees = raw * AS5600_RAW_TO_DEGREES;
  float angle   = degrees - zeroOffset;
  if (angle <    0) angle += 360;
  if (angle >= 360) angle -= 360;

  // ── Timing ─────────────────────────────────────────────
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  prevTime = now;

  // ── PID ────────────────────────────────────────────────
  if (running) {
    error = shortestError(targetAngle, angle);

    if (abs(error) < TOLERANCE) {
      stopMotor();
      running  = false;
      integral = 0;
      Serial.print("✓ Reached "); Serial.print(angle, 1); Serial.println("°");
    }
    else {
      integral   += error * dt;
      integral    = constrain(integral, -100, 100);
      derivative  = (error - prevError) / dt;
      float output = Kp*error + Ki*integral + Kd*derivative;
      prevError   = error;

      pwm = (int)abs(output);
      if (abs(error) < SLOW_ZONE) pwm = pwm * 0.55;
      pwm = constrain(pwm, 0, MAX_PWM);
      if (pwm > 0 && pwm < DEADBAND) pwm = DEADBAND;

      if (output > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
      else            { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
      analogWrite(ENB, pwm);
    }
  }

  // ── Serial Output for Python GUI ───────────────────────
  float P_term = Kp * error;
  float I_term = Ki * integral;
  float D_term = Kd * derivative;

  Serial.print("Angle:");   Serial.print(angle, 1);
  Serial.print(" Target:"); Serial.print(targetAngle, 1);
  Serial.print(" Err:");    Serial.print(error, 1);
  Serial.print(" P:");      Serial.print(P_term, 2);
  Serial.print(" I:");      Serial.print(I_term, 2);
  Serial.print(" D:");      Serial.print(D_term, 2);
  Serial.print(" PWM:");    Serial.println(pwm);

  delay(20);
}
