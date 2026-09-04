/**
  ******************************************************************************
  * @file           : stepper_microstep_ramp_main.c
  * @brief          : STM32F411 + DRV8825 smooth-start microstepping test
  *
  * PA4 -> DIR
  * PA6 -> STEP (TIM3_CH1, AF2)
  *
  * Hardware required before power-on:
  *   - DRV8825 mode DIP switches: ON, ON, ON (1/32 microstep mode)
  *   - nEN/EN held LOW; nRESET and nSLEEP held HIGH
  *   - common ground between BlackPill, driver board, and motor PSU
  *
  * The motor starts at 50 microsteps/s and ramps by only 5 microsteps/s
  * every 50 ms until it reaches 640 microsteps/s.  For a 200-step motor
  * at 1/32 microstepping, that final rate is about 6 RPM.
  ******************************************************************************
  */

#include "main.h"

TIM_HandleTypeDef htim3;

#define TIMER_TICK_HZ      1000000U
#define START_STEP_HZ      50U
#define TARGET_STEP_HZ     640U
#define RAMP_INCREMENT_HZ  5U
#define RAMP_INTERVAL_MS   50U

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void SetStepRate(uint32_t step_hz);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(10);

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Gentle acceleration: no jump to the final STEP frequency. */
  for (uint32_t hz = START_STEP_HZ;
       hz < TARGET_STEP_HZ;
       hz += RAMP_INCREMENT_HZ)
  {
    SetStepRate(hz);
    HAL_Delay(RAMP_INTERVAL_MS);
  }

  SetStepRate(TARGET_STEP_HZ);

  while (1)
  {
    HAL_Delay(100);
  }
}

/* Change the hardware-generated STEP pulse frequency without stopping TIM3. */
static void SetStepRate(uint32_t step_hz)
{
  uint32_t timer_counts = TIMER_TICK_HZ / step_hz;

  __HAL_TIM_SET_AUTORELOAD(&htim3, timer_counts - 1U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, timer_counts / 2U);
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
  rcc_osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK)
  {
    Error_Handler();
  }

  rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  rcc_clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rcc_clk.APB1CLKDivider = RCC_HCLK_DIV1;
  rcc_clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef clock_source = {0};
  TIM_MasterConfigTypeDef master_config = {0};
  TIM_OC_InitTypeDef pwm_config = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15; /* 16 MHz / (15 + 1) = 1 MHz */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (TIMER_TICK_HZ / START_STEP_HZ) - 1U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &clock_source) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  master_config.MasterOutputTrigger = TIM_TRGO_RESET;
  master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master_config) != HAL_OK)
  {
    Error_Handler();
  }

  pwm_config.OCMode = TIM_OCMODE_PWM1;
  pwm_config.Pulse = (TIMER_TICK_HZ / START_STEP_HZ) / 2U;
  pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm_config.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA4: direction output */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PA6: TIM3_CH1 hardware PWM output for STEP */
  gpio.Pin = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
