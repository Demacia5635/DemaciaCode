---
paths:
  - "src/main/java/frc/demacia/utils/mechanisms/**"
---

# Mechanisms (`frc/demacia/utils/mechanisms`)

Base classes that turn "some motors + some sensors" into a WPILib subsystem without boilerplate. Season subsystems (arm, elevator, intake, shooter) should extend these instead of `SubsystemBase` directly.

## `BaseMechanism extends SubsystemBase`

- Constructor: `(String name, MotorInterface[] motors, SensorInterface[] sensors)`.
- Motors and sensors are stored **by name** (`HashMap`) and motors also by index (`motorArray`). Most setters have both forms, for example `setPower("Roller", 0.5)` and `setPower(0, 0.5)`. A wrong name or index is **silently ignored**: no log, no exception. Double-check names.
- Puts per-motor **set brake / set coast** buttons on the dashboard (they work while disabled).
- Helpers: `stopAll()`, `setPowerAll`, `setMotionAll`, `setAngleAll`, `setNeutralModeAll`, `checkElectronicsAll()`, `getMotor(name|index)`, `getSensor(name)`.
- Optional: `withLookUpTable(table, distanceSupplier)`.
- **Calibration gate:** after `withCalibration()`, every motor setter does nothing until `setCalibration(true)` is called (usually by `CalibratinCommand`). If a mechanism "doesn't move", check this first.

## `StateBaseMechanism extends BaseMechanism` — state machine

- Constructor adds the state enum class: `(name, motors, sensors, MyState.class)`. The enum must `implement StateBaseMechanism.MechanismState` (`double[] getValues()`, `String name()`).
- `getValues()` returns **one target per motor, in `motorArray` order**. The array length must match the motor count.
- Built-in states, always present: `IDLE` (all 0, or hold the current position for motors marked with `setPositionMechanism(...)`), `TESTING` (values from the editable dashboard array `"<name>/Test Values"`), and `LOOKUPTABLE` (only after `withLookUpTable`).
- A dashboard `SendableChooser` (`"<name>/State Chooser"`) can switch states by hand. Selecting in the chooser overwrites `state`.
- `setStartingOption(state)` sets the chooser's default.

## Commands in this package

- `DefaultCommand(StateBaseMechanism, ControlMode[])` never ends. Every loop it sends `getValue(i)` to motor `i` using `controlModes[i]` (`DUTYCYCLE`, `VOLTAGE`, `VELOCITY`, `POSITION_VOLTAGE`, `MOTION`, `ANGLE`). Set it as the mechanism's default command.
- `PowerCommand(mechanism, motorName(s), DoubleSupplier(s))`: open-loop power from suppliers (joystick / testing).
- `CalibratinCommand` (sic): drives one motor with `setDuty` (optionally `startPower` for the first `sec` seconds, then `power`) until `stopSupplier` is true (for example a limit switch). In `end()` it stops the motor, sets the encoder to `resetPos` and marks the mechanism calibrated. It does this **even when interrupted**, so an interrupted calibration leaves a wrong zero.

## Rules for new code

- Change state with `mechanism.setState(MyState.X)`, usually inside an `InstantCommand` bound in `RobotContainer`. Don't call motor setters directly while `DefaultCommand` is running, or the two will fight.
- Keep state enums and their numbers in season code (`frc/robot`), not in this package.
- Don't put game-specific logic here. This package must stay reusable.
- Recipe: use the `new-mechanism` skill.
