---
paths:
  - "src/main/java/frc/demacia/utils/motors/**"
  - "src/main/java/frc/demacia/utils/sensors/**"
---

# Motors & sensors (`frc/demacia/utils/motors`, `frc/demacia/utils/sensors`)

The goal of this layer is to make robot code brand-independent. Everything above it uses `MotorInterface` / `SensorInterface`, never vendor classes.

## Motors: Config → `BaseMotor` → vendor subclass

- `BaseMotorConfig<T extends BaseMotorConfig<T>>` is a self-typed builder. Every `withXxx()` returns `T` (`@SuppressWarnings("unchecked") ... return (T) this;`).
- `BaseMotor` (abstract, `implements MotorInterface`) holds **all shared logic**: control modes, units, feed-forward, logging, dashboard tuning, Elastic/SysId registration. Its constructor is a **template method**: `createMotor()` → `configMotor()` (calls the abstract `configXxx` methods, then `applyConfigs()`) → `setSignals()` → `addLog()`.
- `TalonFXMotor`, `TalonSRXMotor`, `SparkMaxMotor` and `SparkFlexMotor` extend `BaseMotor` and **hold** the vendor object (`motor` field). They do not extend it. They only implement the abstract `configXxx` / `setMotorXxx` / `applyXxx` hooks.
- Factory: `config.motorClass.create(config)` (enum `BaseMotorConfig.MotorControllerType`).

Constructor signatures (the name always comes first):

| Config | Constructor |
|--------|-------------|
| `TalonFXConfig` | `(String name, int id, Canbus canbus)` |
| `TalonSRXConfig`, `SparkMaxConfig`, `SparkFlexConfig` | `(String name, int id)` |
| All motor configs | copy: `(String name, int id, BaseMotorConfig<?> other)` |
| `CancoderConfig`, `PigeonConfig` | `(String name, int id, Canbus canbus)` |
| `LimitSwitchConfig`, `AnalogEncoderConfig`, `DigitalEncoderConfig` | `(String name, int channel)` |
| `UltraSonicSensorConfig` | `(String name, int channel, int pingChannel)` |
| `OpticalSensorConfig`, `LidarSensorConfig` | `(String name, int port)` |
| `PneumaticsConfig` | `(String name, int module, PneumaticsModuleType type)` |

`Canbus` is the enum `BaseMotorConfig.Canbus` (`Rio`, `CANIvore`).

## Motor config cheat-sheet

- `withRadiansMotor(gearRatio)` → **radians**. `withMeterMotor(gearRatio, diameterMeters)` → **meters**. `gearRatio` = motor rotations per mechanism rotation. Without either, units are mechanism rotations.
- `withPID(kP, kI, kD, kS, kV, kA, kG, kCos, kV2)` sets slot 0. `withPID(slot, ...)` sets another slot (3 usable slots, 0–2, via `setSlot`). `kCos` = arm gravity term (× cos of the angle). `kV2` = velocity² term. The old `withFeedForward` is gone.
- `withMotionParam(maxVel, maxAccel, maxJerk)` is needed for `setMotion` / `setAngle` (Motion Magic, or MAXMotion on REV).
- Also: `withCurrent(amps)`, `withRampTime(s)`, `withBrake(bool)`, `withInvert(bool)`, `withVolts(max)`, `withCanbus(bus)`.

## What every motor does by itself (don't duplicate)

- Logs position, velocity, acceleration, voltage, current, closed-loop error and set-point into the grouped **file-only** log (metadata `"motors"`, which SysId uses). It also publishes `motors/<name>/wanted value`, `current value` and `is Connected` live.
- Dashboard under `motors/<name>`: PID/FF tuning with an "Update" toggle, Motion Magic tuning, a **test value command** with a control-mode chooser (to run the motor by hand while testing).
- Registers with `ElasticGenerator` (auto dashboard layout) and `Sysid`.

## Gotchas

- Getters return values cached in `Data`. They are refreshed once per loop by `Log.periodic()` → `Data.refreshAll()`. Don't call `refresh()` on signals yourself.
- `setAngle` on a motor without `withRadiansMotor` falls back to `setMotion(angle)` and logs `"cant use setAngle without being in Radians"`. `getCurrentAngle()` is only meaningful for radians motors.
- ControlMode names: `DISABLE, DUTYCYCLE, VOLTAGE, VELOCITY, POSITION_VOLTAGE, MAGIC_MOTION, ANGLE`. It used to be `MOTION`; it is now `MAGIC_MOTION`.
- `isReady(allowedError)` checks closed-loop error. Use it to end position commands.
- **Stall detection is dead config.** `withDetectStallInMotor(...)` stores thresholds and a callback, but nothing reads them.
- `Pneumatics` and `ServoMotor` do **not** implement `SensorInterface`, so they can't go into a `BaseMechanism`.
- Sensors register on the dashboard (`sensors/<name>`) and with Elastic, and log to the file-only group (metadata `"sensors"`).
- Sensor interfaces: `AnalogSensorInterface.get()` returns `double`, `DigitalSensorInterface.get()` returns `boolean`. All of them have `checkElectronics()`.

## Adding a device type

Use the `add-device` skill. For a new motor controller, extend `BaseMotor` and implement its abstract hooks. Don't re-implement control logic.
