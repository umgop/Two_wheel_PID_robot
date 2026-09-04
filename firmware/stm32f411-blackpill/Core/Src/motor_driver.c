#include "motor_driver.h"
#include "robot_config.h"

typedef struct
{
  TIM_HandleTypeDef *timer;
  bool complementary_output;
  bool running;
  bool forward;
  int32_t target_rate_hz;
  int32_t current_rate_hz;
} WheelDrive;

static WheelDrive left_wheel;
static WheelDrive right_wheel;
static bool motors_enabled = false;

static uint32_t AbsoluteRate(int32_t rate_hz)
{
  return (rate_hz < 0) ? (uint32_t)(-rate_hz) : (uint32_t)rate_hz;
}

static int32_t ClampRate(int32_t rate_hz)
{
  if (rate_hz > (int32_t)MAX_STEP_HZ)
  {
    return (int32_t)MAX_STEP_HZ;
  }
  if (rate_hz < -(int32_t)MAX_STEP_HZ)
  {
    return -(int32_t)MAX_STEP_HZ;
  }
  if (AbsoluteRate(rate_hz) < MIN_STEP_HZ)
  {
    return 0;
  }
  return rate_hz;
}

static int32_t RampToward(int32_t current, int32_t target)
{
  const int32_t ramp = (int32_t)RATE_RAMP_HZ_PER_UPDATE;

  /* A sign change always visits zero before a DIR pin changes. */
  if ((current > 0 && target < 0) || (current < 0 && target > 0))
  {
    if (AbsoluteRate(current) <= (uint32_t)ramp)
    {
      return 0;
    }
    return (current > 0) ? (current - ramp) : (current + ramp);
  }

  if (current < target)
  {
    const int32_t next = current + ramp;
    return (next > target) ? target : next;
  }
  if (current > target)
  {
    const int32_t next = current - ramp;
    return (next < target) ? target : next;
  }
  return current;
}

static GPIO_PinState OppositeLevel(GPIO_PinState level)
{
  return (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void SetDirection(bool is_left, bool forward)
{
  const GPIO_PinState forward_level = is_left ? LEFT_FORWARD_STATE : RIGHT_FORWARD_STATE;
  const GPIO_PinState level = forward ? forward_level : OppositeLevel(forward_level);
  HAL_GPIO_WritePin(is_left ? LEFT_DIR_GPIO : RIGHT_DIR_GPIO,
                    is_left ? LEFT_DIR_PIN : RIGHT_DIR_PIN, level);
}

static void StopTimer(WheelDrive *wheel)
{
  if (!wheel->running)
  {
    return;
  }

  if (wheel->complementary_output)
  {
    (void)HAL_TIMEx_PWMN_Stop(wheel->timer, TIM_CHANNEL_1);
  }
  else
  {
    (void)HAL_TIM_PWM_Stop(wheel->timer, TIM_CHANNEL_1);
  }
  wheel->running = false;
}

static void ConfigureTimerRate(WheelDrive *wheel, uint32_t step_hz)
{
  uint32_t counts = (TIMER_TICK_HZ + (step_hz / 2U)) / step_hz;

  if (counts < 4U)
  {
    counts = 4U;
  }

  __HAL_TIM_SET_AUTORELOAD(wheel->timer, counts - 1U);
  __HAL_TIM_SET_COMPARE(wheel->timer, TIM_CHANNEL_1, counts / 2U);
}

static void StartTimer(WheelDrive *wheel)
{
  if (HAL_TIM_GenerateEvent(wheel->timer, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_SET_COUNTER(wheel->timer, 0U);

  HAL_StatusTypeDef status = wheel->complementary_output
      ? HAL_TIMEx_PWMN_Start(wheel->timer, TIM_CHANNEL_1)
      : HAL_TIM_PWM_Start(wheel->timer, TIM_CHANNEL_1);

  if (status != HAL_OK)
  {
    Error_Handler();
  }

  wheel->running = true;
}

static void ApplyWheelRate(WheelDrive *wheel, bool is_left)
{
  const uint32_t magnitude = AbsoluteRate(wheel->current_rate_hz);

  if (magnitude < MIN_STEP_HZ)
  {
    StopTimer(wheel);
    return;
  }

  const bool forward = wheel->current_rate_hz > 0;
  if (wheel->running && forward != wheel->forward)
  {
    StopTimer(wheel);
  }

  if (!wheel->running)
  {
    SetDirection(is_left, forward);
    /* Exceeds the DRV8825 DIR-to-STEP setup time by a wide margin. */
    HAL_Delay(1);
    wheel->forward = forward;
    ConfigureTimerRate(wheel, magnitude);
    StartTimer(wheel);
    return;
  }

  ConfigureTimerRate(wheel, magnitude);
}

void MotorDriver_Init(TIM_HandleTypeDef *left_timer,
                      TIM_HandleTypeDef *right_timer)
{
  left_wheel.timer = left_timer;
  left_wheel.complementary_output = false;
  left_wheel.running = false;
  left_wheel.forward = true;
  left_wheel.target_rate_hz = 0;
  left_wheel.current_rate_hz = 0;

  right_wheel.timer = right_timer;
  right_wheel.complementary_output = true;
  right_wheel.running = false;
  right_wheel.forward = true;
  right_wheel.target_rate_hz = 0;
  right_wheel.current_rate_hz = 0;

  HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO, MOTOR_ENABLE_PIN, MOTOR_ENABLE_IDLE_STATE);
  SetDirection(true, true);
  SetDirection(false, true);
}

void MotorDriver_SetEnabled(bool enabled)
{
  if (!enabled)
  {
    MotorDriver_Stop();
  }

  HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO, MOTOR_ENABLE_PIN,
                    enabled ? MOTOR_ENABLE_ACTIVE_STATE : MOTOR_ENABLE_IDLE_STATE);
  motors_enabled = enabled;
}

bool MotorDriver_IsEnabled(void)
{
  return motors_enabled;
}

void MotorDriver_SetWheelRates(int32_t left_step_hz, int32_t right_step_hz)
{
  left_wheel.target_rate_hz = ClampRate(left_step_hz);
  right_wheel.target_rate_hz = ClampRate(right_step_hz);
}

void MotorDriver_Update(void)
{
  if (!motors_enabled)
  {
    return;
  }

  left_wheel.current_rate_hz = RampToward(left_wheel.current_rate_hz,
                                           left_wheel.target_rate_hz);
  right_wheel.current_rate_hz = RampToward(right_wheel.current_rate_hz,
                                            right_wheel.target_rate_hz);

  ApplyWheelRate(&left_wheel, true);
  ApplyWheelRate(&right_wheel, false);
}

void MotorDriver_Stop(void)
{
  left_wheel.target_rate_hz = 0;
  left_wheel.current_rate_hz = 0;
  right_wheel.target_rate_hz = 0;
  right_wheel.current_rate_hz = 0;
  StopTimer(&left_wheel);
  StopTimer(&right_wheel);
}

void MotorDriver_EmergencyStop(void)
{
  MotorDriver_Stop();
  MotorDriver_SetEnabled(false);
}

int32_t MotorDriver_LeftRate(void)
{
  return left_wheel.current_rate_hz;
}

int32_t MotorDriver_RightRate(void)
{
  return right_wheel.current_rate_hz;
}
