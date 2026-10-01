---
paths:
  - "src/main/java/frc/demacia/utils/motors/**"
  - "src/main/java/frc/demacia/utils/sensors/**"
---

# Motors & sensors (`frc/demacia/utils/motors`, `frc/demacia/utils/sensors`)

The goal of this layer is to make robot code brand-independent. Everything above it uses `MotorInterface` / `SensorInterface`, never vendor classes.

## Pattern: Config → Device → Interface

- Each device has a `XxxConfig` builder and a wrapper class that **extends the vendor class** and **implements the Demacia interface**. Example: `TalonFXMotor extends TalonFX implements MotorInterface`.
- Configs use a self-typed generic, `BaseMotorConfig<T extends BaseMotorConfig<T>>` (and `BaseSensorConfig<T>`). Every `withXxx()` returns `T`, so chains keep the concrete type. New `withXxx()` methods must follow this: `@SuppressWarnings("unchecked") ... return (T) this;`
- Factory: `config.motorClass.create(config)` (enum `BaseMotorConfig.MotorControllerType`) builds the right wrapper.

Constructor signatures differ by device, so check them before writing code:

| Config | Constructor |
|--------|-------------|
| `TalonFXConfig` | `(int id, Canbus canbus, String name)` |
| `TalonSRXConfig`, `SparkMaxConfig`, `SparkFlexConfig` | `(int id, String name)` |
| All motor configs | copy: `(int id, String name, BaseMotorConfig<?> other)` |
| `CancoderConfig`, `PigeonConfig` | `(int id, Canbus canbus, String name)` |
| `LimitSwitchConfig`, `AnalogEncoderConfig`, `DigitalEncoderConfig` | `(int channel, String name)` |
| `UltraSonicSensorConfig` | `(int channel, int pingChannel, String name)` |
| `OpticalSensorConfig`, `LidarSensorConfig` | `(String name, int port)` |

`Canbus` is the enum `BaseMotorConfig.Canbus` (`Rio`, `CANIvore`).

## Motor config cheat-sheet

- `withRadiansMotor(gearRatio)` → positions in **radians**. `withMeterMotor(gearRatio, diameterMeters)` → **meters**. `gearRatio` = motor rotations per mechanism rotation. Without either, units are mechanism rotations.
- `withPID(kP, kI, kD, kS, kV, kA, kG)` sets slot 0. `withPID(slot, ...)` sets another slot. Switch slots at runtime with `changeSlot(slot)`.
- `withMotionParam(maxVel, maxAccel, maxJerk)` is **required** before `setMotion` / `setAngle`. SparkMax logs an error and does nothing if `maxVelocity == 0`.
- `withFeedForward(kv2, kSin)`: `kv2` is a velocity² term, `kSin` is a gravity term for arms (cos of the angle).
- Also: `withCurrent(amps)` (supply limit), `withRampTime(s)`, `withBrake(bool)`, `withInvert(bool)`, `withVolts(max)`, `withMaxPositionError(err)`.

## What a wrapper constructor already does

It applies the config, creates its signals, **registers its own log entries** (position, velocity, acceleration, voltage, current, set-point, error) and sets its dashboard name. Don't add duplicate log entries for these values.

## Gotchas

- **Cached values:** getters (`getCurrentPosition()`, ...) return values cached in `Data`. They are refreshed once per loop by `LogManager.periodic()` → `Data.refreshAll()`, which bulk-refreshes all Phoenix 6 signals. Don't call `refresh()` on signals yourself in periodic code.
- `getCurrentAngle()` returns **0** unless the motor was configured with `withRadiansMotor`.
- `setAngle(rad)` takes the shortest path (`angleModulus`) and then calls `setMotion`.
- **TalonSRX has no motion profiling.** `setMotion` / `setAngle` only log `"there is no motion"`. Use `setPositionVoltage` instead.
- **Stall detection never fires on its own.** `withDetectStallInMotor(current, velocity, seconds, callback)` stores thresholds, but nothing in the code calls `updateStallDetection()`. Call it from the owning subsystem's `periodic()`.
- `SparkMaxMotor.changeSlot` accepts 0–3 but maps 3 to slot 2.
- `Pneumatics` and `ServoMotor` have configs but do **not** implement `SensorInterface`, so they can't go into a `BaseMechanism` sensor array.
- Sensor interfaces: `AnalogSensorInterface.get()` returns `double`, `DigitalSensorInterface.get()` returns `boolean`. All sensors have `checkElectronics()`.

## Adding a new device type

Use the `add-device` skill. In short: config class + wrapper class + (for motors) a new `MotorControllerType` enum entry + an `addLog()` that registers through `LogManager`.
