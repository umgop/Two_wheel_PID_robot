#ifndef IOT_UART_H
#define IOT_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

typedef struct
{
  bool valid;
  bool set_enabled;
  bool enabled;
  bool set_target_pitch;
  float target_pitch_deg;
  bool set_turn;
  int32_t turn_step_hz;
} IotCommand;

void IotUart_Init(UART_HandleTypeDef *huart);
bool IotUart_GetCommand(IotCommand *command);
void IotUart_SendTelemetry(bool enabled,
                           float pitch_deg,
                           float target_pitch_deg,
                           int32_t left_step_hz,
                           int32_t right_step_hz);

#endif /* IOT_UART_H */
