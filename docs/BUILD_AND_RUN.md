# Build and run

1. Import `firmware/stm32f411-blackpill` into STM32CubeIDE.
2. Verify the target part is `STM32F411CEUx` and build the project.
3. Flash via SWD: `SWDIO → PA13`, `SWCLK → PA14`, and `GND → GND`.
4. Power the BlackPill from a known-good source; a damaged ST-Link 3.3 V rail
   must not be used as a target supply.
5. Apply motor power, keeping the chassis supported. The `PB5` enable line is
   high (disabled) until an `ENABLE` command arrives.
6. Hold the robot upright and still for the calibration period after boot.
7. Confirm incoming telemetry before sending `ENABLE`.

The PID values and polarity settings live in
`Core/Inc/robot_config.h`. If a physical change reverses a wheel or IMU axis,
correct the named configuration value rather than swapping unknown motor wires.

For standalone GNU Arm builds, ensure `-lm` is present because the pitch filter
uses `atan2f()` and `sqrtf()`.
