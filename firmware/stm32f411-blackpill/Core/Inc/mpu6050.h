#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include "main.h"

typedef struct
{
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
} Mpu6050Sample;

bool Mpu6050_Init(I2C_HandleTypeDef *hi2c);
bool Mpu6050_Read(Mpu6050Sample *sample);

#endif /* MPU6050_H */
