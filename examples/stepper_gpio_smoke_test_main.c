/**
  ******************************************************************************
  * @file           : stepper_gpio_smoke_test_main.c
  * @brief          : STM32F411 + DRV8825 wiring smoke test
  *
  * PA4 -> DIR
  * PA6 -> STEP
  *
  * This intentionally does NOT use TIM3/PWM or microstepping.  It emits
  * 20 full-step pulses per second, runs 200 pulses, pauses, reverses, and
  * repeats.  It is a hardware/wiring test, not final motion-control firmware.
  ******************************************************************************
  */

#include "main.h"

#define DIR_PORT        GPIOA
#define DIR_PIN         GPIO_PIN_4
#define STEP_PORT       GPIOA
#define STEP_PIN        GPIO_PIN_6

#define STEP_HIGH_MS    2U
#define STEP_LOW_MS     48U
#define STEPS_PER_MOVE  200U

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void StepOnce(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* Give the driver a defined direction before the first STEP edge. */
  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
  HAL_Delay(10);

  while (1)
  {
    /* 200 full steps = one revolution for a 1.8-degree NEMA 17 motor. */
    for (uint16_t step = 0; step < STEPS_PER_MOVE; ++step)
    {
      StepOnce();
    }

    HAL_Delay(500);
    HAL_GPIO_TogglePin(DIR_PORT, DIR_PIN);
    HAL_Delay(10);
  }
}

static void StepOnce(void)
{
  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
  HAL_Delay(STEP_HIGH_MS);
  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
  HAL_Delay(STEP_LOW_MS);
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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, DIR_PIN | STEP_PIN, GPIO_PIN_RESET);

  gpio.Pin = DIR_PIN | STEP_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
