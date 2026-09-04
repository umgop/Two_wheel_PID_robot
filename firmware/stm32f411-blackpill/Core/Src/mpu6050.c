#include "mpu6050.h"

#define MPU6050_ADDRESS             (0x68U << 1)
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_WHO_AM_I        0x75U

#define MPU6050_ACCEL_LSB_PER_G     16384.0f
#define MPU6050_GYRO_LSB_PER_DPS    131.0f

static I2C_HandleTypeDef *mpu_i2c = NULL;

static bool WriteRegister(uint8_t register_address, uint8_t value)
{
  return HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDRESS, register_address,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U) == HAL_OK;
}

bool Mpu6050_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t who_am_i = 0U;

  mpu_i2c = hi2c;
  HAL_Delay(100);

  if (HAL_I2C_IsDeviceReady(mpu_i2c, MPU6050_ADDRESS, 3U, 100U) != HAL_OK)
  {
    return false;
  }

  if (HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, 100U) != HAL_OK ||
      who_am_i != 0x68U)
  {
    return false;
  }

  if (!WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x80U))
  {
    return false;
  }
  HAL_Delay(100);

  /* Wake the IMU using the gyro PLL. */
  if (!WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x01U))
  {
    return false;
  }

  /* 200 Hz output with a 42 Hz digital low-pass filter. */
  if (!WriteRegister(MPU6050_REG_SMPLRT_DIV, 4U) ||
      !WriteRegister(MPU6050_REG_CONFIG, 0x03U) ||
      !WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x00U) ||
      !WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x00U))
  {
    return false;
  }

  return true;
}

bool Mpu6050_Read(Mpu6050Sample *sample)
{
  uint8_t raw[14];

  if (mpu_i2c == NULL || sample == NULL ||
      HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 20U) != HAL_OK)
  {
    return false;
  }

  const int16_t ax = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  const int16_t ay = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
  const int16_t az = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
  const int16_t gx = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
  const int16_t gy = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
  const int16_t gz = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

  sample->ax_g = (float)ax / MPU6050_ACCEL_LSB_PER_G;
  sample->ay_g = (float)ay / MPU6050_ACCEL_LSB_PER_G;
  sample->az_g = (float)az / MPU6050_ACCEL_LSB_PER_G;
  sample->gx_dps = (float)gx / MPU6050_GYRO_LSB_PER_DPS;
  sample->gy_dps = (float)gy / MPU6050_GYRO_LSB_PER_DPS;
  sample->gz_dps = (float)gz / MPU6050_GYRO_LSB_PER_DPS;

  return true;
}
