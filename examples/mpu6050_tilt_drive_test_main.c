/**
  ******************************************************************************
  * @file           : mpu6050_tilt_drive_test_main.c
  * @brief          : Conservative MPU-6050 tilt-to-drive test (no PID)
  *
  * STM32F411 BlackPill wiring:
  *   PA4 -> left DRV8825 DIR       PA6 -> left DRV8825 STEP  (TIM3_CH1)
  *   PA5 -> right DRV8825 DIR      PA7 -> right DRV8825 STEP (TIM3_CH2)
  *   PB6 -> MPU-6050 SCL (I2C1)    PB7 -> MPU-6050 SDA (I2C1)
  *
  * This is NOT a self-balancing controller.  It is only a safe bench test:
  *   - during the first 3 seconds, keep the robot still and level;
  *   - within +/- 7 degrees of that position, the STEP outputs are stopped;
  *   - a gentle forward/backward tilt commands both wheels at the same rate;
  *   - reversing tilt always ramps to a stop before reversing wheel direction.
  *
  * Mount the sensor solidly with its X axis pointing toward the robot front
  * and its Z axis upward.  If forward tilt drives backward, change
  * TILT_FORWARD_SIGN from +1.0f to -1.0f.
  ******************************************************************************
  */

#include "main.h"
#include <stdbool.h>
#include <math.h>

TIM_HandleTypeDef htim3;
I2C_HandleTypeDef hi2c1;

/* ---------- Pins and motor settings ---------- */
#define LEFT_DIR_PIN          GPIO_PIN_4
#define RIGHT_DIR_PIN         GPIO_PIN_5

/* Set either definition to GPIO_PIN_RESET if that wheel needs inversion. */
#define LEFT_DIR_FORWARD      GPIO_PIN_SET
#define RIGHT_DIR_FORWARD     GPIO_PIN_SET

#define TIMER_TICK_HZ         1000000U  /* HSI 16 MHz / (15 + 1) */
#define MIN_RUN_STEP_HZ       120U
#define MAX_STEP_HZ           1200U     /* 11.25 RPM at 1/32, 200-step motor */
#define RAMP_STEP_HZ          20U       /* 20 Hz every 10 ms = 2000 Hz/s */

/* ---------- Tilt command settings ---------- */
#define CONTROL_PERIOD_MS     10U       /* 100 Hz sensor/control loop */
#define DEAD_BAND_DEG         7.0f
#define FULL_TILT_DEG         22.0f
#define TILT_FORWARD_SIGN     1.0f

/* First test is accelerometer-only.  Set to 1 only after axis/sign are proven. */
#define USE_GYRO_COMPLEMENTARY_FILTER 0U

/* Complementary filter: gyro gives short-term response, accel removes drift. */
#define FILTER_GYRO_WEIGHT    0.98f
#define ACCEL_LOW_PASS_WEIGHT 0.85f
#define RAD_TO_DEG            57.2957795f
#define ACCEL_VALID_MIN_G      0.85f
#define ACCEL_VALID_MAX_G      1.15f

/* ---------- MPU-6050 registers ---------- */
#define MPU6050_ADDRESS       (0x68U << 1) /* AD0 connected to GND */
#define MPU6050_REG_SMPLRT    0x19U
#define MPU6050_REG_CONFIG    0x1AU
#define MPU6050_REG_GYRO_CFG  0x1BU
#define MPU6050_REG_ACCEL_CFG 0x1CU
#define MPU6050_REG_ACCEL_XH  0x3BU
#define MPU6050_REG_PWR_MGMT1 0x6BU
#define MPU6050_REG_WHO_AM_I  0x75U

#define MPU6050_ACCEL_LSB_G   16384.0f  /* +/- 2 g */
#define MPU6050_GYRO_LSB_DPS  131.0f    /* +/- 250 degrees/s */

#define CALIBRATION_SAMPLES   300U      /* 3 seconds while motionless */

typedef struct
{
  float ax_g;
  float ay_g;
  float az_g;
  float gy_dps;
} MpuSample;

static bool step_output_running = false;
static bool active_direction_forward = true;
static uint32_t current_step_hz = 0U;
#if USE_GYRO_COMPLEMENTARY_FILTER
static float gyro_y_bias_dps = 0.0f;
#endif
static float filtered_pitch_deg = 0.0f;
static float neutral_pitch_deg = 0.0f;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static bool MPU6050_Init(void);
static bool MPU6050_Read(MpuSample *sample);
static bool MPU6050_Calibrate(void);
static float AccelPitchDeg(const MpuSample *sample);
static float AccelMagnitudeG(const MpuSample *sample);
static void SetStepRate(uint32_t step_hz);
static void SetDriveDirection(bool forward);
static void StartStepping(bool forward);
static void StopStepping(void);
static void UpdateDrive(uint32_t requested_step_hz, bool requested_forward);
static uint32_t TiltToStepRate(float tilt_deg, bool *forward);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();

  /* No STEP PWM is started until calibration and a deliberate tilt command. */
  if (!MPU6050_Init())
  {
    Error_Handler();
  }

  /* Keep the chassis still and in its desired neutral orientation now. */
  if (!MPU6050_Calibrate())
  {
    Error_Handler();
  }

#if USE_GYRO_COMPLEMENTARY_FILTER
  uint32_t last_ms = HAL_GetTick();
#endif

  while (1)
  {
    MpuSample sample;
    if (!MPU6050_Read(&sample))
    {
      /* A lost sensor must never leave the robot driving. */
      StopStepping();
      Error_Handler();
    }

    /* Pitch: X points forward, Y is the pitch rotation axis, Z points up. */
    float accel_pitch_deg = AccelPitchDeg(&sample);
    /* Do not let a burst of chassis acceleration look like a tilt angle. */
    float magnitude_g = AccelMagnitudeG(&sample);
    if ((magnitude_g >= ACCEL_VALID_MIN_G) &&
        (magnitude_g <= ACCEL_VALID_MAX_G))
    {
#if USE_GYRO_COMPLEMENTARY_FILTER
      uint32_t now_ms = HAL_GetTick();
      float dt_s = (float)(now_ms - last_ms) * 0.001f;
      last_ms = now_ms;
      if ((dt_s <= 0.0f) || (dt_s > 0.05f))
      {
        dt_s = (float)CONTROL_PERIOD_MS * 0.001f;
      }

      float gyro_y_dps = sample.gy_dps - gyro_y_bias_dps;
      filtered_pitch_deg += gyro_y_dps * dt_s;
      filtered_pitch_deg = (FILTER_GYRO_WEIGHT * filtered_pitch_deg) +
                           ((1.0f - FILTER_GYRO_WEIGHT) * accel_pitch_deg);
#else
      /* Simple low-pass accelerometer angle: ideal for proving wiring/axes. */
      filtered_pitch_deg = (ACCEL_LOW_PASS_WEIGHT * filtered_pitch_deg) +
                           ((1.0f - ACCEL_LOW_PASS_WEIGHT) * accel_pitch_deg);
#endif
    }

    float control_tilt_deg =
        (filtered_pitch_deg - neutral_pitch_deg) * TILT_FORWARD_SIGN;

    bool request_forward = true;
    uint32_t requested_step_hz =
        TiltToStepRate(control_tilt_deg, &request_forward);
    UpdateDrive(requested_step_hz, request_forward);

    HAL_Delay(CONTROL_PERIOD_MS);
  }
}

/* Convert the filtered forward/back pitch into one shared wheel speed. */
static uint32_t TiltToStepRate(float tilt_deg, bool *forward)
{
  float magnitude = fabsf(tilt_deg);

  if (magnitude <= DEAD_BAND_DEG)
  {
    return 0U;
  }

  *forward = (tilt_deg > 0.0f);

  float fraction = (magnitude - DEAD_BAND_DEG) /
                   (FULL_TILT_DEG - DEAD_BAND_DEG);
  if (fraction > 1.0f)
  {
    fraction = 1.0f;
  }

  return MIN_RUN_STEP_HZ +
         (uint32_t)(fraction * (float)(MAX_STEP_HZ - MIN_RUN_STEP_HZ));
}

/* Decelerate before a reversal; the two timers channels always use one rate. */
static void UpdateDrive(uint32_t requested_step_hz, bool requested_forward)
{
  if (requested_step_hz == 0U)
  {
    if (!step_output_running)
    {
      return;
    }

    if (current_step_hz > (MIN_RUN_STEP_HZ + RAMP_STEP_HZ))
    {
      current_step_hz -= RAMP_STEP_HZ;
      SetStepRate(current_step_hz);
    }
    else
    {
      StopStepping();
    }
    return;
  }

  if (!step_output_running)
  {
    StartStepping(requested_forward);
    return;
  }

  if (requested_forward != active_direction_forward)
  {
    /* Brake to zero first; the next loop will apply the new direction. */
    if (current_step_hz > (MIN_RUN_STEP_HZ + RAMP_STEP_HZ))
    {
      current_step_hz -= RAMP_STEP_HZ;
      SetStepRate(current_step_hz);
    }
    else
    {
      StopStepping();
    }
    return;
  }

  if (current_step_hz < requested_step_hz)
  {
    uint32_t next = current_step_hz + RAMP_STEP_HZ;
    current_step_hz = (next > requested_step_hz) ? requested_step_hz : next;
    SetStepRate(current_step_hz);
  }
  else if (current_step_hz > requested_step_hz)
  {
    uint32_t next = current_step_hz - RAMP_STEP_HZ;
    current_step_hz = (next < requested_step_hz) ? requested_step_hz : next;
    SetStepRate(current_step_hz);
  }
}

static void SetDriveDirection(bool forward)
{
  GPIO_PinState left = forward ? LEFT_DIR_FORWARD :
                    ((LEFT_DIR_FORWARD == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  GPIO_PinState right = forward ? RIGHT_DIR_FORWARD :
                     ((RIGHT_DIR_FORWARD == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOA, LEFT_DIR_PIN, left);
  HAL_GPIO_WritePin(GPIOA, RIGHT_DIR_PIN, right);

  /* DRV8825 needs only sub-microsecond DIR setup; 2 ms is deliberately safe. */
  HAL_Delay(2);
}

static void StartStepping(bool forward)
{
  SetDriveDirection(forward);
  SetStepRate(MIN_RUN_STEP_HZ);

  if ((HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK))
  {
    Error_Handler();
  }

  active_direction_forward = forward;
  current_step_hz = MIN_RUN_STEP_HZ;
  step_output_running = true;
}

static void StopStepping(void)
{
  if (step_output_running)
  {
    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  }

  current_step_hz = 0U;
  step_output_running = false;
}

static void SetStepRate(uint32_t step_hz)
{
  uint32_t counts = TIMER_TICK_HZ / step_hz;

  __HAL_TIM_SET_AUTORELOAD(&htim3, counts - 1U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, counts / 2U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, counts / 2U);

  /* Load the preshadowed ARR/CCR before starting after a stopped state. */
  if (!step_output_running)
  {
    __HAL_TIM_GENERATE_EVENT(&htim3, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
  }
}

static bool MPU6050_Init(void)
{
  uint8_t value;
  uint8_t who_am_i = 0U;

  HAL_Delay(100); /* Let the sensor and its breakout-board regulator settle. */

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS, 3U, 100U) != HAL_OK)
  {
    return false;
  }

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, 100U) != HAL_OK)
  {
    return false;
  }
  if (who_am_i != 0x68U)
  {
    return false;
  }

  value = 0x80U; /* Reset, so an earlier firmware run cannot leave stale setup. */
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT1,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return false;
  }
  HAL_Delay(100);

  value = 0x01U; /* Wake up and use the gyro PLL clock. */
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT1,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return false;
  }

  value = 9U;    /* 1 kHz / (1 + 9) = 100 Hz when DLPF is enabled. */
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_SMPLRT,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return false;
  }

  value = 0x03U; /* DLPF about 42-44 Hz: useful against motor vibration. */
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_CONFIG,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return false;
  }

  value = 0x00U; /* gyro +/-250 dps, accel +/-2 g */
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_GYRO_CFG,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return false;
  }
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_ACCEL_CFG,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) != HAL_OK)
  {
    return false;
  }

  return true;
}

static bool MPU6050_Read(MpuSample *sample)
{
  uint8_t raw[14];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_REG_ACCEL_XH,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 20U) != HAL_OK)
  {
    return false;
  }

  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);

  sample->ax_g = (float)ax / MPU6050_ACCEL_LSB_G;
  sample->ay_g = (float)ay / MPU6050_ACCEL_LSB_G;
  sample->az_g = (float)az / MPU6050_ACCEL_LSB_G;
  sample->gy_dps = (float)gy / MPU6050_GYRO_LSB_DPS;
  return true;
}

static float AccelPitchDeg(const MpuSample *sample)
{
  float denominator = sqrtf((sample->ay_g * sample->ay_g) +
                            (sample->az_g * sample->az_g));
  return atan2f(-sample->ax_g, denominator) * RAD_TO_DEG;
}

static float AccelMagnitudeG(const MpuSample *sample)
{
  return sqrtf((sample->ax_g * sample->ax_g) +
               (sample->ay_g * sample->ay_g) +
               (sample->az_g * sample->az_g));
}

static bool MPU6050_Calibrate(void)
{
  float angle_sum = 0.0f;
#if USE_GYRO_COMPLEMENTARY_FILTER
  float gyro_sum = 0.0f;
#endif

  for (uint32_t i = 0U; i < CALIBRATION_SAMPLES; ++i)
  {
    MpuSample sample;
    if (!MPU6050_Read(&sample))
    {
      return false;
    }

    angle_sum += AccelPitchDeg(&sample);
#if USE_GYRO_COMPLEMENTARY_FILTER
    gyro_sum += sample.gy_dps;
#endif
    HAL_Delay(CONTROL_PERIOD_MS);
  }

  neutral_pitch_deg = angle_sum / (float)CALIBRATION_SAMPLES;
  filtered_pitch_deg = neutral_pitch_deg;
#if USE_GYRO_COMPLEMENTARY_FILTER
  gyro_y_bias_dps = gyro_sum / (float)CALIBRATION_SAMPLES;
#endif
  return true;
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
  uint32_t start_counts = TIMER_TICK_HZ / MIN_RUN_STEP_HZ;

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = start_counts - 1U;
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
  pwm_config.Pulse = start_counts / 2U;
  pwm_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm_config.OCFastMode = TIM_OCFAST_DISABLE;

  if ((HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_config, TIM_CHANNEL_2) != HAL_OK))
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000U;
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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA4/PA5 are the left/right direction outputs. */
  HAL_GPIO_WritePin(GPIOA, LEFT_DIR_PIN | RIGHT_DIR_PIN, GPIO_PIN_RESET);
  gpio.Pin = LEFT_DIR_PIN | RIGHT_DIR_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PA6 = TIM3_CH1 and PA7 = TIM3_CH2, the two matching STEP streams. */
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PB6/PB7 are I2C1 SCL/SDA, AF4, open drain.  MPU module pulls to 3.3 V. */
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio);

  __HAL_RCC_I2C1_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
