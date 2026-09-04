# MQTT protocol

The ESP32 subscribes to the command topic declared in its local `secrets.h`.
It forwards each payload as a single line to the STM32 over UART2.

## Commands

| MQTT payload | Result |
|---|---|
| `ENABLE` | Enables the calibrated balance loop and driver outputs |
| `DISABLE` | Stops pulses and disables the drivers |
| `STOP` | Alias for `DISABLE` |
| `SETPOINT 0.35` | Sets the target pitch in degrees, limited to ±4° |
| `TURN 250` | Applies a differential wheel command in step-Hz, limited to ±700 |

## Telemetry

The STM32 emits this comma-separated line at 10 Hz:

```text
STATE,<enabled>,<pitch_cdeg>,<target_cdeg>,<left_step_hz>,<right_step_hz>
```

For example:

```text
STATE,1,-18,35,420,395
```

means the controller is enabled, pitch is -0.18°, target is +0.35°, and the
left/right motor rates are 420/395 step pulses per second.

Do not commit `secrets.h`, broker credentials, certificates, or private IP
addresses.
