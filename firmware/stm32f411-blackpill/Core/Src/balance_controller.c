#include "balance_controller.h"
#include "robot_config.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f

static float AccelerometerPitchDeg(const Mpu6050Sample *sample)
{
  const float denominator = sqrtf((sample->ay_g * sample->ay_g) +
                                  (sample->az_g * sample->az_g));
  return atan2f(-sample->ax_g, denominator) * RAD_TO_DEG;
}

static float AccelerationMagnitudeG(const Mpu6050Sample *sample)
{
  return sqrtf((sample->ax_g * sample->ax_g) +
               (sample->ay_g * sample->ay_g) +
               (sample->az_g * sample->az_g));
}

bool BalanceController_Calibrate(BalanceController *controller)
{
  float accel_angle_sum = 0.0f;
  float gyro_y_sum = 0.0f;

  if (controller == NULL)
  {
    return false;
  }

  for (uint32_t i = 0U; i < CALIBRATION_SAMPLES; ++i)
  {
    Mpu6050Sample sample;
    if (!Mpu6050_Read(&sample))
    {
      return false;
    }

    const float magnitude = AccelerationMagnitudeG(&sample);
    if (magnitude < ACCEL_VALID_MIN_G || magnitude > ACCEL_VALID_MAX_G ||
        fabsf(sample.gy_dps) > 3.0f)
    {
      return false;
    }

    accel_angle_sum += AccelerometerPitchDeg(&sample);
    gyro_y_sum += sample.gy_dps;
    HAL_Delay(CONTROL_PERIOD_MS);
  }

  controller->accel_pitch_offset_deg = accel_angle_sum / (float)CALIBRATION_SAMPLES;
  controller->gyro_y_bias_dps = gyro_y_sum / (float)CALIBRATION_SAMPLES;
  controller->pitch_deg = 0.0f;
  controller->pitch_rate_dps = 0.0f;
  controller->target_pitch_deg = TARGET_PITCH_DEG;
  controller->integral = 0.0f;
  controller->calibrated = true;
  return true;
}

void BalanceController_Reset(BalanceController *controller)
{
  if (controller != NULL)
  {
    controller->integral = 0.0f;
  }
}

void BalanceController_SetTargetPitch(BalanceController *controller,
                                      float target_pitch_deg)
{
  if (controller == NULL)
  {
    return;
  }

  if (target_pitch_deg > MAX_REMOTE_TARGET_DEG)
  {
    target_pitch_deg = MAX_REMOTE_TARGET_DEG;
  }
  else if (target_pitch_deg < -MAX_REMOTE_TARGET_DEG)
  {
    target_pitch_deg = -MAX_REMOTE_TARGET_DEG;
  }
  controller->target_pitch_deg = target_pitch_deg;
}

bool BalanceController_Update(BalanceController *controller,
                              const Mpu6050Sample *sample,
                              float dt_s,
                              int32_t *base_step_rate_hz)
{
  if (controller == NULL || sample == NULL || base_step_rate_hz == NULL ||
      !controller->calibrated || dt_s <= 0.0f || dt_s > 0.02f)
  {
    return false;
  }

  const float accel_pitch = PITCH_SIGN *
      (AccelerometerPitchDeg(sample) - controller->accel_pitch_offset_deg);
  const float gyro_pitch_rate = PITCH_SIGN *
      (sample->gy_dps - controller->gyro_y_bias_dps);
  const float predicted_pitch = controller->pitch_deg + (gyro_pitch_rate * dt_s);
  const float magnitude = AccelerationMagnitudeG(sample);

  if (magnitude >= ACCEL_VALID_MIN_G && magnitude <= ACCEL_VALID_MAX_G)
  {
    controller->pitch_deg = ((1.0f - ACCEL_CORRECTION_WEIGHT) * predicted_pitch) +
                            (ACCEL_CORRECTION_WEIGHT * accel_pitch);
  }
  else
  {
    controller->pitch_deg = predicted_pitch;
  }
  controller->pitch_rate_dps = gyro_pitch_rate;

  if (fabsf(controller->pitch_deg) > FALL_ANGLE_DEG)
  {
    return false;
  }

  const float error = controller->target_pitch_deg - controller->pitch_deg;
  controller->integral += error * dt_s;
  if (controller->integral > PID_INTEGRAL_LIMIT)
  {
    controller->integral = PID_INTEGRAL_LIMIT;
  }
  else if (controller->integral < -PID_INTEGRAL_LIMIT)
  {
    controller->integral = -PID_INTEGRAL_LIMIT;
  }

  float output = (PID_KP * error) +
                 (PID_KI * controller->integral) -
                 (PID_KD * controller->pitch_rate_dps);
  output *= MOTOR_OUTPUT_SIGN;

  if (output > (float)MAX_STEP_HZ)
  {
    output = (float)MAX_STEP_HZ;
  }
  else if (output < -(float)MAX_STEP_HZ)
  {
    output = -(float)MAX_STEP_HZ;
  }

  *base_step_rate_hz = (int32_t)output;
  return true;
}
