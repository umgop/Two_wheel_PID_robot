# Bring-up examples

These conservative single-purpose files are retained from hardware bring-up.
They are not part of the finished balance firmware.

- `stepper_gpio_smoke_test_main.c` — verifies one driver/motor coil pairing.
- `stepper_microstep_ramp_main.c` — verifies a gentle 1/32 microstep ramp.
- `mpu6050_tilt_drive_test_main.c` — verifies MPU wiring and tilt direction
  before enabling PID.

Run them only with the matching pin configuration and a supported chassis.
