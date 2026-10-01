---
paths:
  - "src/main/java/frc/demacia/utils/controller/**"
  - "src/main/java/frc/demacia/utils/leds/**"
  - "src/main/java/frc/demacia/utils/geometry/**"
  - "src/main/java/frc/demacia/utils/*.java"
---

# Controllers, LEDs, math helpers, geometry

## `CommandController` (`utils/controller`)

- `new CommandController(port, ControllerType.kXbox | kPS5)`. A single class with the same method names for both controllers: `upButton()` (Y / Triangle), `downButton()`, `leftButton()`, `rightButton()`, `leftBumper()`, `rightBumper()`, `leftStick()`, `rightStick()`, `leftSettings()`, `rightSetting()`, `getPS()`, `getTouchPad()`.
- Axes come **already dead-banded**: `getLeftX/Y()`, `getRightX/Y()`, and `getLeftTrigger()` / `getRightTrigger()` in 0..1 (the PS5 −1..1 trigger range is converted). Deadbands are in `ControllerConstants`. Don't apply a second deadband.
- `getLeftX(threshold)` etc. return a `Trigger` that fires when the axis passes the threshold.

## LEDs (`utils/leds`)

- Create exactly **one** `LedManager` (it owns the single `AddressableLED` on PWM `LedConstants.PORT` = 9, length `LedConstants.LENGTH` = 43). Split it into named `LedStrip(name, size, ledManager[, offset])` segments.
- Strip API: `setColor(Color | Color[])`, `setBlink(...)`, `setGay()` (animated rainbow; rename to `setRainbow` when you touch it), `turnOff()`.
- The roboRIO supports only one addressable LED output, so never create a second `LedManager`.

## Math helpers (`utils/*.java`)

- `LookUpTable(columns)` or `LookUpTable(double[][])`, then `add(key, v1, v2, ...)`. `get(key)` returns the `columns − 1` values, **linearly interpolated**, clamped to the first/last row outside the range, and `null` if the table is empty. Rows are kept sorted by key.
- `Trapezoid.calculate(currentV, wantedV, maxV, accel, distanceLeft)` returns the next velocity of a trapezoid profile. It assumes a ~0.03 s step.
- `DemaciaUtils`: global `getIsRed()` / `getIsComp()` suppliers, set once from `RobotContainer`.
- `Utils` contains old season leftovers (`seeNote`, shooting tables, commented speaker/amp code) mixed with general helpers. `Utilities` only has `deadband`. In new code, put general helpers in one class and season helpers in `frc/robot`.

## Geometry (`utils/geometry/*Demacia`)

- Copies of WPILib `Pose2d`, `Translation2d`, `Rotation2d`, `Transform2d`, `Field2d`, `FieldObject2d`.
- ⚠️ `Pose2dDemacia`, `Translation2dDemacia`, `Rotation2dDemacia` and `Transform2dDemacia` are **mutable**: `plus()`, `minus()`, `rotateBy()` change the object itself, unlike WPILib's immutable classes. Never share an instance between subsystems or store one you got from someone else without copying it.
- Nothing outside `utils/geometry` uses them today. Prefer WPILib's immutable classes unless you have a measured performance reason.
