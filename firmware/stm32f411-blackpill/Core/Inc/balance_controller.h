#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "mpu6050.h"

typedef struct
{
  float pitch_deg;
  float pitch_rate_dps;
  float target_pitch_deg;
  float integral;
  float gyro_y_bias_dps;
  float accel_pitch_offset_deg;
  bool calibrated;
} BalanceController;

bool BalanceController_Calibrate(BalanceController *controller);
void BalanceController_Reset(BalanceController *controller);
void BalanceController_SetTargetPitch(BalanceController *controller,
                                      float target_pitch_deg);
bool BalanceController_Update(BalanceController *controller,
                              const Mpu6050Sample *sample,
                              float dt_s,
                              int32_t *base_step_rate_hz);

#endif /* BALANCE_CONTROLLER_H */
