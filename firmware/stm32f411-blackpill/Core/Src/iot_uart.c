#include "iot_uart.h"
#include "robot_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_LINE_LENGTH 64U

static UART_HandleTypeDef *iot_uart = NULL;
static uint8_t received_byte;
static char assembling_line[RX_LINE_LENGTH];
static char completed_line[RX_LINE_LENGTH];
static volatile uint32_t assembling_length = 0U;
static volatile bool line_ready = false;

void IotUart_Init(UART_HandleTypeDef *huart)
{
  iot_uart = huart;
  assembling_length = 0U;
  line_ready = false;

  if (HAL_UART_Receive_IT(iot_uart, &received_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != iot_uart)
  {
    return;
  }

  const char character = (char)received_byte;
  if (character == '\n')
  {
    if (!line_ready && assembling_length > 0U)
    {
      assembling_line[assembling_length] = '\0';
      memcpy(completed_line, assembling_line, assembling_length + 1U);
      line_ready = true;
    }
    assembling_length = 0U;
  }
  else if (character != '\r' && !line_ready)
  {
    if (assembling_length < (RX_LINE_LENGTH - 1U))
    {
      assembling_line[assembling_length++] = character;
    }
    else
    {
      assembling_length = 0U;
    }
  }

  (void)HAL_UART_Receive_IT(iot_uart, &received_byte, 1U);
}

bool IotUart_GetCommand(IotCommand *command)
{
  char line[RX_LINE_LENGTH];
  uint32_t primask;

  if (command == NULL)
  {
    return false;
  }

  memset(command, 0, sizeof(*command));

  primask = __get_PRIMASK();
  __disable_irq();
  if (!line_ready)
  {
    __set_PRIMASK(primask);
    return false;
  }
  memcpy(line, completed_line, sizeof(line));
  line_ready = false;
  __set_PRIMASK(primask);

  if (strcmp(line, "ENABLE") == 0)
  {
    command->valid = true;
    command->set_enabled = true;
    command->enabled = true;
  }
  else if (strcmp(line, "DISABLE") == 0 || strcmp(line, "STOP") == 0)
  {
    command->valid = true;
    command->set_enabled = true;
    command->enabled = false;
  }
  else if (strncmp(line, "SETPOINT ", 9U) == 0)
  {
    command->valid = true;
    command->set_target_pitch = true;
    command->target_pitch_deg = strtof(&line[9], NULL);
  }
  else if (strncmp(line, "TURN ", 5U) == 0)
  {
    command->valid = true;
    command->set_turn = true;
    command->turn_step_hz = (int32_t)strtol(&line[5], NULL, 10);
  }

  return command->valid;
}

void IotUart_SendTelemetry(bool enabled,
                           float pitch_deg,
                           float target_pitch_deg,
                           int32_t left_step_hz,
                           int32_t right_step_hz)
{
  char message[96];
  const int32_t pitch_cdeg = (int32_t)(pitch_deg * 100.0f);
  const int32_t target_cdeg = (int32_t)(target_pitch_deg * 100.0f);
  const int count = snprintf(message, sizeof(message),
                             "STATE,%u,%ld,%ld,%ld,%ld\n",
                             enabled ? 1U : 0U,
                             (long)pitch_cdeg,
                             (long)target_cdeg,
                             (long)left_step_hz,
                             (long)right_step_hz);

  if (iot_uart != NULL && count > 0)
  {
    const uint16_t length = (count < (int)sizeof(message))
        ? (uint16_t)count : (uint16_t)(sizeof(message) - 1U);
    (void)HAL_UART_Transmit(iot_uart, (uint8_t *)message, length, 10U);
  }
}
