/*
 * Simplified Arduino two-wheel PID balancing controller.
 *
 * Hardware:
 *   Arduino A4 -> MPU6050 SDA       Arduino A5 -> MPU6050 SCL
 *   D2 -> left STEP, D3 -> left DIR
 *   D4 -> right STEP, D5 -> right DIR
 *   D8 -> both DRV8825 nEN (active LOW)
 *
 * The MPU6050 must run at 3.3 V logic levels.  Set the DRV8825 current limit
 * first, keep the chassis supported for its first tests, and never hot-plug a
 * stepper motor while the driver has motor power.
 *
 * Serial commands at 115200 baud:
 *   e = enable balancing
 *   x = stop / disable motors
 */

#include <Wire.h>
#include <math.h>

constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t LEFT_STEP_PIN = 2;
constexpr uint8_t LEFT_DIR_PIN = 3;
constexpr uint8_t RIGHT_STEP_PIN = 4;
constexpr uint8_t RIGHT_DIR_PIN = 5;
constexpr uint8_t MOTOR_ENABLE_PIN = 8;

constexpr bool LEFT_FORWARD_HIGH = true;
constexpr bool RIGHT_FORWARD_HIGH = true;

constexpr unsigned long CONTROL_PERIOD_US = 5000UL;  // 200 Hz
constexpr int MIN_STEP_HZ = 90;
constexpr int MAX_STEP_HZ = 2400;
constexpr int MAX_TURN_STEP_HZ = 400;
constexpr float FALL_ANGLE_DEG = 32.0f;

/* Starter tuning for this simplified Arduino implementation. */
constexpr float KP = 235.0f;
constexpr float KI = 18.0f;
constexpr float KD = 7.2f;
constexpr float INTEGRAL_LIMIT = 10.0f;
constexpr float TARGET_PITCH_DEG = 0.35f;
constexpr float ACCEL_WEIGHT = 0.02f;

struct ImuSample {
  float ax_g;
  float ay_g;
  float az_g;
  float gy_dps;
};

struct Wheel {
  uint8_t stepPin;
  uint8_t dirPin;
  bool forwardHigh;
  int rateHz;
  unsigned long lastStepUs;
};

Wheel leftWheel  = {LEFT_STEP_PIN, LEFT_DIR_PIN, LEFT_FORWARD_HIGH, 0, 0};
Wheel rightWheel = {RIGHT_STEP_PIN, RIGHT_DIR_PIN, RIGHT_FORWARD_HIGH, 0, 0};

float gyroBiasDps = 0.0f;
float accelPitchOffsetDeg = 0.0f;
float pitchDeg = 0.0f;
float integral = 0.0f;
bool balanceEnabled = false;
int turnStepHz = 0;
unsigned long lastControlUs = 0;

int16_t readInt16() {
  const uint8_t highByte = Wire.read();
  const uint8_t lowByte = Wire.read();
  return (int16_t)((uint16_t(highByte) << 8) | lowByte);
}

bool readMpu(ImuSample &sample) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) {
    return false;
  }

  const int16_t ax = readInt16();
  const int16_t ay = readInt16();
  const int16_t az = readInt16();
  (void)readInt16();  // temperature
  (void)readInt16();  // gyro X
  const int16_t gy = readInt16();
  (void)readInt16();  // gyro Z

  sample.ax_g = ax / 16384.0f;
  sample.ay_g = ay / 16384.0f;
  sample.az_g = az / 16384.0f;
  sample.gy_dps = gy / 131.0f;
  return true;
}

float accelerometerPitchDeg(const ImuSample &sample) {
  return atan2f(-sample.ax_g,
                sqrtf(sample.ay_g * sample.ay_g + sample.az_g * sample.az_g)) *
         57.2957795f;
}

void setMotorsEnabled(bool enabled) {
  digitalWrite(MOTOR_ENABLE_PIN, enabled ? LOW : HIGH);
  if (!enabled) {
    leftWheel.rateHz = 0;
    rightWheel.rateHz = 0;
  }
}

void setWheelRate(Wheel &wheel, int requestedRateHz) {
  requestedRateHz = constrain(requestedRateHz, -MAX_STEP_HZ, MAX_STEP_HZ);
  if (abs(requestedRateHz) < MIN_STEP_HZ) {
    requestedRateHz = 0;
  }

  if (requestedRateHz != 0) {
    const bool forward = requestedRateHz > 0;
    const bool dirLevel = forward ? wheel.forwardHigh : !wheel.forwardHigh;
    digitalWrite(wheel.dirPin, dirLevel ? HIGH : LOW);
  }
  wheel.rateHz = requestedRateHz;
}

void serviceWheel(Wheel &wheel, unsigned long nowUs) {
  const int magnitude = abs(wheel.rateHz);
  if (magnitude == 0) {
    return;
  }

  const unsigned long periodUs = 1000000UL / (unsigned long)magnitude;
  if ((unsigned long)(nowUs - wheel.lastStepUs) >= periodUs) {
    digitalWrite(wheel.stepPin, HIGH);
    delayMicroseconds(4);  // longer than the DRV8825 minimum STEP pulse width
    digitalWrite(wheel.stepPin, LOW);
    wheel.lastStepUs = nowUs;
  }
}

void serviceSteppers() {
  const unsigned long nowUs = micros();
  serviceWheel(leftWheel, nowUs);
  serviceWheel(rightWheel, nowUs);
}

bool calibrateImu() {
  float pitchSum = 0.0f;
  float gyroSum = 0.0f;

  Serial.println(F("Hold robot upright and still: calibrating MPU6050..."));
  for (int i = 0; i < 600; ++i) {
    ImuSample sample;
    if (!readMpu(sample) || fabsf(sample.gy_dps) > 3.0f) {
      return false;
    }
    pitchSum += accelerometerPitchDeg(sample);
    gyroSum += sample.gy_dps;
    delay(5);
  }

  accelPitchOffsetDeg = pitchSum / 600.0f;
  gyroBiasDps = gyroSum / 600.0f;
  pitchDeg = 0.0f;
  integral = 0.0f;
  return true;
}

void processSerialCommand() {
  if (!Serial.available()) {
    return;
  }

  const char command = (char)Serial.read();
  if (command == 'e' || command == 'E') {
    integral = 0.0f;
    balanceEnabled = true;
    setMotorsEnabled(true);
    Serial.println(F("Balancing enabled"));
  } else if (command == 'x' || command == 'X') {
    balanceEnabled = false;
    setMotorsEnabled(false);
    Serial.println(F("Motors disabled"));
  }
}

void setup() {
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  setMotorsEnabled(false);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01);  // wake MPU6050, gyro PLL clock
  if (Wire.endTransmission() != 0 || !calibrateImu()) {
    Serial.println(F("MPU6050 calibration failed; motors remain disabled."));
    while (true) {
      delay(1000);
    }
  }

  lastControlUs = micros();
  Serial.println(F("Ready. Send e to enable, x to stop."));
}

void loop() {
  processSerialCommand();
  serviceSteppers();

  const unsigned long nowUs = micros();
  if ((unsigned long)(nowUs - lastControlUs) < CONTROL_PERIOD_US) {
    return;
  }

  const float dt = (nowUs - lastControlUs) * 0.000001f;
  lastControlUs = nowUs;

  ImuSample sample;
  if (!readMpu(sample)) {
    balanceEnabled = false;
    setMotorsEnabled(false);
    return;
  }

  const float accelPitch = accelerometerPitchDeg(sample) - accelPitchOffsetDeg;
  const float pitchRate = sample.gy_dps - gyroBiasDps;
  const float predictedPitch = pitchDeg + pitchRate * dt;
  const float magnitudeG = sqrtf(sample.ax_g * sample.ax_g +
                                 sample.ay_g * sample.ay_g +
                                 sample.az_g * sample.az_g);

  if (magnitudeG > 0.82f && magnitudeG < 1.18f) {
    pitchDeg = (1.0f - ACCEL_WEIGHT) * predictedPitch + ACCEL_WEIGHT * accelPitch;
  } else {
    pitchDeg = predictedPitch;
  }

  if (!balanceEnabled) {
    return;
  }

  if (fabsf(pitchDeg) > FALL_ANGLE_DEG) {
    balanceEnabled = false;
    setMotorsEnabled(false);
    Serial.println(F("Fall detected; motors disabled."));
    return;
  }

  const float error = TARGET_PITCH_DEG - pitchDeg;
  integral = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float output = KP * error + KI * integral - KD * pitchRate;
  output = constrain(output, -(float)MAX_STEP_HZ, (float)MAX_STEP_HZ);

  const int baseRateHz = (int)output;
  setWheelRate(leftWheel, baseRateHz - turnStepHz);
  setWheelRate(rightWheel, baseRateHz + turnStepHz);
}
