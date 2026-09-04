#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/* TIM3 CH1 drives PA6; TIM1 CH1N drives PA7. */
void MotorDriver_Init(TIM_HandleTypeDef *left_timer,
                      TIM_HandleTypeDef *right_timer);
void MotorDriver_SetEnabled(bool enabled);
bool MotorDriver_IsEnabled(void);
void MotorDriver_SetWheelRates(int32_t left_step_hz,
                                int32_t right_step_hz);
void MotorDriver_Update(void);
void MotorDriver_Stop(void);
void MotorDriver_EmergencyStop(void);
int32_t MotorDriver_LeftRate(void);
int32_t MotorDriver_RightRate(void);

#endif /* MOTOR_DRIVER_H */
