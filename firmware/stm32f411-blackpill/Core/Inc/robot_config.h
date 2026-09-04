#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/*
 * Final reference tuning and pin configuration.
 *
 * The IMU is mounted with X pointing toward the robot front and Z upward.
 * Positive logical wheel speed means "drive the robot forward"; the two
 * DIR levels can be different because the motors are mirror-mounted.
 */

#include "main.h"

/* DRV8825 control pins. */
#define LEFT_DIR_GPIO              GPIOA
#define LEFT_DIR_PIN               GPIO_PIN_4
#define RIGHT_DIR_GPIO             GPIOA
#define RIGHT_DIR_PIN              GPIO_PIN_5

/* Shared nEN line: active LOW.  A 10 kOhm pull-up keeps motors disabled
 * while the MCU is resetting. */
#define MOTOR_ENABLE_GPIO          GPIOB
#define MOTOR_ENABLE_PIN           GPIO_PIN_5
#define MOTOR_ENABLE_ACTIVE_STATE  GPIO_PIN_RESET
#define MOTOR_ENABLE_IDLE_STATE    GPIO_PIN_SET

/* Change only one of these if the chassis turns instead of driving straight. */
#define LEFT_FORWARD_STATE         GPIO_PIN_SET
#define RIGHT_FORWARD_STATE        GPIO_PIN_SET

/* PA6 / TIM3_CH1 produces left STEP; PA7 / TIM1_CH1N produces right STEP. */
#define TIMER_TICK_HZ              1000000U
#define MIN_STEP_HZ                90U
#define MAX_STEP_HZ                4000U
#define RATE_RAMP_HZ_PER_UPDATE    75U

/* Controller rate and MPU-6050 calibration. */
#define CONTROL_PERIOD_MS          5U
#define CONTROL_PERIOD_S           0.005f
#define CALIBRATION_SAMPLES        600U
#define ACCEL_CORRECTION_WEIGHT    0.02f
#define ACCEL_VALID_MIN_G          0.82f
#define ACCEL_VALID_MAX_G          1.18f
#define FALL_ANGLE_DEG             32.0f

/* Final PID values for the documented chassis.  Units are step-Hz/degree. */
#define PID_KP                     235.0f
#define PID_KI                     18.0f
#define PID_KD                     7.2f
#define PID_INTEGRAL_LIMIT         10.0f

/* Mechanical and sensor polarity calibration. */
#define TARGET_PITCH_DEG           0.35f
#define PITCH_SIGN                 1.0f
#define MOTOR_OUTPUT_SIGN          1.0f
#define MAX_REMOTE_TARGET_DEG      4.0f
#define MAX_TURN_STEP_HZ           700

/* UART2 provides the 3.3 V serial link to the ESP32 MQTT bridge. */
#define ROBOT_UART_BAUD            115200U
#define TELEMETRY_INTERVAL_MS      100U

#endif /* ROBOT_CONFIG_H */
