<img width="4032" height="3024" alt="IMG_8823" src="https://github.com/user-attachments/assets/cf864d6b-59d8-4923-a98f-cc50dc89ba86" />
# Two-Wheel Self-Balancing Robot

This is the finished reference project for a two-wheel self-balancing robot.
It combines an STM32F411 BlackPill, two DRV8825 stepper drivers, an MPU6050,
a tuned complementary-filter/PID balance loop, and an ESP32 MQTT bridge for
IoT control and telemetry.

The final STM32 configuration uses independent STEP generators so the PID can
apply a turn correction to either wheel:

```text
MPU6050 pitch + gyro rate
          │
          ▼
  complementary filter (200 Hz)
          │
          ▼
        PID output
          │
          ├── left wheel rate  = balance rate - turn command
          └── right wheel rate = balance rate + turn command
```

## Repository layout

```text
firmware/
├── stm32f411-blackpill/     Complete STM32CubeIDE project
├── arduino_two_wheel_pid/   Small, standalone Arduino learning version
└── esp32_mqtt_bridge/       MQTT-to-STM32 UART bridge
examples/                    Earlier motor and IMU bring-up tests
docs/                        Wiring, MQTT protocol, and build notes
```

## Finished features

- MPU6050 calibration, complementary pitch filter, fall detection, and
  safety stop
- Tuned PID constants in
  [`robot_config.h`](firmware/stm32f411-blackpill/Core/Inc/robot_config.h)
- Independent left/right step rates with ramped reversal
- Separate direction polarity for mirror-mounted wheels
- Shared active-low driver enable with a safe disabled-at-reset state
- ESP32 Wi-Fi/MQTT command bridge and STM32 telemetry stream
- Simplified Arduino two-wheel PID implementation for experimentation

## Wiring summary

| BlackPill pin | Connection | Function |
|---|---|---|
| `PA4` | Left DRV8825 `DIR` | Left wheel direction |
| `PA6` | Left DRV8825 `STEP` | Left pulses, `TIM3_CH1` |
| `PA5` | Right DRV8825 `DIR` | Right wheel direction |
| `PA7` | Right DRV8825 `STEP` | Right pulses, `TIM1_CH1N` |
| `PB5` | Both driver `nEN` pins | Motor enable, active-low |
| `PB6` | MPU6050 `SCL` | I2C1 clock |
| `PB7` | MPU6050 `SDA` | I2C1 data |
| `PA2` | ESP32 UART RX (`GPIO16`) | IoT telemetry TX |
| `PA3` | ESP32 UART TX (`GPIO17`) | IoT command RX |
| `3V3` | MPU6050 `VCC` | IMU logic power |
| `GND` | MPU6050, drivers, ESP32, supply negative | Common reference |

The MPU6050 `AD0` pin is tied low for address `0x68`. Its `INT`, `XCL`, and
`XDA` pins are not needed here. Mount the sensor rigidly with its **X axis
pointing forward** and **Z axis upward**.

For the documented StepperOnline `17HE15-1504S` motors, use these coil pairs:

```text
Black + Blue  = coil A
Green + Red   = coil B
```

For a driver socket labelled `1A 1B 2A 2B`, the order is normally
`Black Blue Green Red`. Keep every coil pair together even if a connector's
physical wire order is different.

Full wiring and power rules are in [docs/WIRING.md](docs/WIRING.md).

## Power and safety

The motor supply is separate from the MCU/IMU supply. Use a 12 V motor supply
for the DRV8825 `VMOT` inputs and power the BlackPill from USB or a known-good
regulated 5 V input. The MPU6050 is powered from **3.3 V only**.

Never connect 5 V to `PA4`–`PA7`, `PB5`–`PB7`, MPU6050 `SDA`, or MPU6050
`SCL`. Set the DRV8825 current limit before use and never hot-plug a motor
while its driver has motor power.

Before first motion, keep the wheels off the ground or restrain the chassis.
The firmware starts disabled; send `ENABLE` only after the MPU6050 finishes
its three-second still calibration.

## STM32 build

1. Open `firmware/stm32f411-blackpill` in STM32CubeIDE.
2. Build the `TwoWheelSelfBalancingRobot` project and flash it through SWD.
3. Keep the robot upright and motionless during the initial calibration.
4. Send `ENABLE` through the ESP32 MQTT bridge or a 115200-baud UART terminal.
5. Send `DISABLE`/`STOP` to stop and disable both drivers.

The STM32 project includes the HAL/CMSIS sources and links the math library.
Do not commit generated `Debug/` files.

## MQTT / IoT

Copy:

```text
firmware/esp32_mqtt_bridge/secrets.example.h
```

to `secrets.h`, add local Wi-Fi and MQTT values, install the **PubSubClient**
Arduino library, and upload `Esp32MqttBridge.ino` to the ESP32. Details and
payload formats are in [docs/MQTT_PROTOCOL.md](docs/MQTT_PROTOCOL.md).

## Arduino version

[`TwoWheelPidController.ino`](firmware/arduino_two_wheel_pid/TwoWheelPidController.ino)
is a simplified learning implementation. It uses an Arduino's `Wire` library,
two STEP/DIR pairs, and serial `e`/`x` commands. It is intentionally smaller
than the STM32 production firmware and does not include the MQTT bridge.


