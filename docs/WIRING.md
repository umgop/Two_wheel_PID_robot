# Wiring and power

## Motor controller wiring

| Signal | Left driver | Right driver |
|---|---|---|
| `DIR` | BlackPill `PA4` | BlackPill `PA5` |
| `STEP` | BlackPill `PA6` | BlackPill `PA7` |
| `nEN` | BlackPill `PB5` | BlackPill `PB5` |
| Logic GND | BlackPill GND | BlackPill GND |
| Motor power | 12 V `VMOT` / supply negative | 12 V `VMOT` / supply negative |

`nEN` is active-low. Add a 10 kOhm pull-up from the shared `nEN` line to 3.3 V
so the drivers begin disabled until firmware deliberately enables them.
`nSLEEP` and `nRESET` must be held HIGH. Configure the three microstep switches
the same on both drivers; this build uses the expansion-board `ON ON ON`
setting for 1/32 microstepping.

The right STEP wire stays on `PA7`; the finished firmware configures it as
`TIM1_CH1N`, not `TIM3_CH2`, so the two wheels can run at independent rates.

## MPU6050 wiring

| MPU6050 | BlackPill |
|---|---|
| `VCC` | `3V3` |
| `GND` | `GND` |
| `SCL` | `PB6` |
| `SDA` | `PB7` |
| `AD0` | `GND` |
| `INT`, `XCL`, `XDA` | Not connected |

Use a breakout with 3.3 V I2C pull-ups, or add suitable pull-ups to 3.3 V.
Do not power its bus at 5 V.

## ESP32 bridge wiring

| ESP32 | BlackPill |
|---|---|
| `GPIO17` / TX2 | `PA3` / USART2 RX |
| `GPIO16` / RX2 | `PA2` / USART2 TX |
| `GND` | `GND` |

Both UARTs are 3.3 V logic and use 115200 baud.

## Power checklist

1. Use a separate 12 V supply for motor power. Add the driver carrier's
   recommended bulk capacitor near each `VMOT` connection.
2. Power the BlackPill through USB or a verified regulated 5 V input.
3. Power the MPU6050 from the BlackPill 3.3 V rail.
4. Join the BlackPill ground, MPU ground, both driver logic grounds, ESP32
   ground, and motor-supply negative.
5. Set each DRV8825 current limit for the actual motor and carrier board before
   enabling it.

Never attach or detach a motor while the 12 V driver supply is connected.
