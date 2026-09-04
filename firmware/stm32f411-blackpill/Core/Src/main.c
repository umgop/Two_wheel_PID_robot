/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Two-wheel STM32F411 self-balancing robot firmware
  *
  * Left motor:  PA4 DIR, PA6 STEP (TIM3_CH1 / AF2)
  * Right motor: PA5 DIR, PA7 STEP (TIM1_CH1N / AF1)
  * Shared nEN:  PB5 (active LOW)
  * MPU-6050:    PB6 SCL, PB7 SDA (I2C1, 3.3 V only)
  * ESP32 link:  PA2 TX, PA3 RX (USART2, 115200 baud)
  ******************************************************************************
  */

#include "main.h"
#include "balance_controller.h"
#include "iot_uart.h"
#include "motor_driver.h"
#include "mpu6050.h"
#include "robot_config.h"
#include <stdbool.h>

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static int32_t ClampTurnRate(int32_t value);

int main(void)
{
  BalanceController balance = {0};
  bool balance_enabled = false;
  int32_t turn_step_hz = 0;
  uint32_t last_control_ms;
  uint32_t last_telemetry_ms;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  MotorDriver_Init(&htim3, &htim1);
  IotUart_Init(&huart2);

  if (!Mpu6050_Init(&hi2c1))
  {
    Error_Handler();
  }

  /* Keep the robot upright and still for this three-second calibration. */
  if (!BalanceController_Calibrate(&balance))
  {
    Error_Handler();
  }

  last_control_ms = HAL_GetTick();
  last_telemetry_ms = last_control_ms;

  while (1)
  {
    IotCommand command;
    if (IotUart_GetCommand(&command))
    {
      if (command.set_target_pitch)
      {
        BalanceController_SetTargetPitch(&balance, command.target_pitch_deg);
      }
      if (command.set_turn)
      {
        turn_step_hz = ClampTurnRate(command.turn_step_hz);
      }
      if (command.set_enabled)
      {
        balance_enabled = command.enabled;
        if (balance_enabled)
        {
          BalanceController_Reset(&balance);
          MotorDriver_SetEnabled(true);
        }
        else
        {
          turn_step_hz = 0;
          MotorDriver_EmergencyStop();
        }
      }
    }

    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_control_ms) >= CONTROL_PERIOD_MS)
    {
      const float dt_s = (float)(now_ms - last_control_ms) * 0.001f;
      last_control_ms = now_ms;

      Mpu6050Sample sample;
      if (!Mpu6050_Read(&sample))
      {
        balance_enabled = false;
        turn_step_hz = 0;
        MotorDriver_EmergencyStop();
      }
      else if (balance_enabled)
      {
        int32_t base_step_hz = 0;
        if (!BalanceController_Update(&balance, &sample, dt_s, &base_step_hz))
        {
          /* A fall or invalid control state requires an explicit re-enable. */
          balance_enabled = false;
          turn_step_hz = 0;
          MotorDriver_EmergencyStop();
        }
        else
        {
          const int32_t left_step_hz = base_step_hz - turn_step_hz;
          const int32_t right_step_hz = base_step_hz + turn_step_hz;
          MotorDriver_SetWheelRates(left_step_hz, right_step_hz);
          MotorDriver_Update();
        }
      }
    }

    if ((now_ms - last_telemetry_ms) >= TELEMETRY_INTERVAL_MS)
    {
      last_telemetry_ms = now_ms;
      IotUart_SendTelemetry(balance_enabled, balance.pitch_deg,
                            balance.target_pitch_deg,
                            MotorDriver_LeftRate(), MotorDriver_RightRate());
    }
  }
}

static int32_t ClampTurnRate(int32_t value)
{
  if (value > MAX_TURN_STEP_HZ)
  {
    return MAX_TURN_STEP_HZ;
  }
  if (value < -MAX_TURN_STEP_HZ)
  {
    return -MAX_TURN_STEP_HZ;
  }
  return value;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef rcc_osc = {0};
  RCC_ClkInitTypeDef rcc_clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  rcc_osc.HSIState = RCC_HSI_ON;
  rcc_osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  rcc_osc.PLL.PLLState = RCC_PLL_ON;
  rcc_osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  rcc_osc.PLL.PLLM = 16U;
  rcc_osc.PLL.PLLN = 336U;
  rcc_osc.PLL.PLLP = RCC_PLLP_DIV4;
  rcc_osc.PLL.PLLQ = 7U;
  if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK)
  {
    Error_Handler();
  }

  rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rcc_clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rcc_clk.APB1CLKDivider = RCC_HCLK_DIV2;
  rcc_clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void ConfigurePwmTimer(TIM_HandleTypeDef *timer)
{
  TIM_ClockConfigTypeDef clock_source = {0};
  TIM_MasterConfigTypeDef master_config = {0};
  TIM_OC_InitTypeDef pwm_config = {0};
  const uint32_t initial_counts = TIMER_TICK_HZ / MIN_STEP_HZ;

  timer->Init.Prescaler = 83U; /* Both selected timer clocks are 84 MHz. */
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = initial_counts - 1U;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(timer) != HAL_OK)
  {
    Error_Handler();
  }

  clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(timer, &clock_source) != HAL_OK ||
      HAL_TIM_PWM_Init(timer) != HAL_OK)
  {
    Error_Handler();
  }

  master_config.MasterOutputTrigger = TIM_TRGO_RESET;
  master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(timer, &master_config) != HAL_OK)
  {
    Error_Handler();
  }

  pwm_config.OCMode = TIM_OCMODE_PWM1;
  pwm_config.Pulse = initial_counts / 2U;
  pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  pwm_config.OCFastMode = TIM_OCFAST_DISABLE;
  pwm_config.OCIdleState = TIM_OCIDLESTATE_RESET;
  pwm_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(timer, &pwm_config, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  htim1.Instance = TIM1;
  ConfigurePwmTimer(&htim1);
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM3_Init(void)
{
  htim3.Instance = TIM3;
  ConfigurePwmTimer(&htim3);
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000U;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = ROBOT_UART_BAUD;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, LEFT_DIR_PIN | RIGHT_DIR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO, MOTOR_ENABLE_PIN, MOTOR_ENABLE_IDLE_STATE);

  gpio.Pin = LEFT_DIR_PIN | RIGHT_DIR_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = MOTOR_ENABLE_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_ENABLE_GPIO, &gpio);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
