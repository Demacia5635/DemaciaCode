---
paths:
  - "src/main/java/frc/demacia/utils/mechanisms/**"
---

# Mechanisms (`frc/demacia/utils/mechanisms`)

Base classes that turn "some motors + some sensors" into a WPILib subsystem without boilerplate. Season subsystems (arm, elevator, intake, shooter) should extend these instead of `SubsystemBase` directly.

## `BaseMechanism extends SubsystemBase`

- Constructor: `(String name, MotorInterface[] motors, SensorInterface[] sensors)`. Either array may be `null`.
- Each motor is wrapped in a `MotorNode` (motor + `minLimit`/`maxLimit` + `hasCalibrated` + `autoCalibration`), stored by name. Index versions look the name up in `motorNames[]`.
- Setters: `setPower`, `setVoltage`, `setVelocity`, `setPositionVoltage`, `setMotion`, `setAngle` (by name or index), `setPowerAll`, `stop()`, `setNeutralMode(...)`. A wrong motor name is **silently ignored** in setters. `stop` / `addLimit` log `"Invalid motor"`.
- Helpers: `isReady(name|index, allowedError)`, `isReady(double[] errors)`, `checkElectronics()`, `getMotor`, `getSensor`, `getMotors`, `getSensors`.
- Dashboard: brake/coast buttons per motor and for the whole mechanism. `withPowerCommand(motorName, DoubleSupplier)` adds a manual-power button. The mechanism registers itself with `ElasticGenerator`.

### Limits and calibration (per motor)

- `addLimit(motor, min, max)`, `addLimitMin`, `addLimitMax`: targets of `setPositionVoltage`, `setMotion` and `setAngle` are **clamped** into the limits. Power, voltage and velocity are not limited.
- `withCalibration(motor)` marks the motor uncalibrated. **Only the position setters** (`setPositionVoltage`, `setMotion`, `setAngle`) do nothing until `setCalibration(motor, true)`. Power, voltage and velocity still work, so calibration can drive the motor.
- `withAutoCalibration(motor, BooleanSupplier atLimit, resetPos)` calibrates automatically: when `atLimit` becomes true, it sets the encoder to `resetPos`. It also adds a "manual reset" dashboard button. **It runs from `BaseMechanism.periodic()`**, so a subclass that overrides `periodic()` must call `super.periodic()`.
- `BaseMechanism.setAngle` is **not** the motor's `setAngle`. It clamps (with wrap-around) into the limits and then calls `motor.setMotion`. Without limits, the angle is used as an absolute position, with no shortest-path wrap.

## `StateBaseMechanism extends BaseMechanism` — state machine

- Constructor: `(name, motors, sensors, MyState.class)`. The enum implements `StateBaseMechanism.MechanismState` (`double[] getValues()`, `String name()`).
- `getValues()` = **one target per motor, in constructor order**. The array length must match the motor count.
- Built-in states: `IDLE_STATE` (the default) and `TESTING_STATE` (values from the dashboard array `"<name> Test Values"`). `setStateIdle()` / `setStateTesting()` switch to them. The old `LOOKUPTABLE` state and `setPositionMechanism` no longer exist.
- Dashboard chooser `"<name>/<name> State Chooser"`. Selecting an option overwrites `state`. `setStartingOption(state)` changes the chooser default.

## Commands

- `DefaultCommand(StateBaseMechanism, ControlMode[])` never ends. **In `IDLE` it calls `mechanism.stop()`**. In any other state it sends `getValue(i)` to motor `i` with `controlModes[i]` (`DUTYCYCLE`, `VOLTAGE`, `VELOCITY`, `POSITION_VOLTAGE`, `MAGIC_MOTION`, `ANGLE`).
- `PowerCommand(mechanism, motorName(s), DoubleSupplier(s))`: open-loop power, which stops the motors on `end`.
- `CalibrationCommand(mechanism, motorName, power, stopSupplier, resetPos[, startPower, sec])`: drives one motor with `setDuty` until `stopSupplier` is true. Then it stops, sets the encoder to `resetPos`, and marks that motor calibrated. ⚠️ It does this **even when interrupted**, which leaves a wrong zero. Prefer `withAutoCalibration` where a limit switch exists.

## Rules for new code

- Change state with `mechanism.setState(MyState.X)` inside an `InstantCommand` **without** requiring the mechanism. Requiring it would interrupt a running calibration.
- Keep state enums and their numbers in season code (`frc/robot`). Keep this package game-agnostic.
- Note: `BaseMechanism.periodic()` publishes values with `SmartDashboard.putNumber/putBoolean` every loop. That is old style; new code should use `Log.putData`.
- Recipe: use the `new-mechanism` skill.
