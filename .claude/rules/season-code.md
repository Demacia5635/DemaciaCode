---
paths:
  - "src/main/java/frc/robot/**"
---

# Season code (`frc/robot`)

This is the only place for game-specific code. The library under `frc/demacia` is shared across seasons and may be overwritten by the sync workflow.

## Files and their jobs

- `Main.java`: entry point. Don't touch it.
- `Robot.java` (`TimedRobot`): runs `CommandScheduler.getInstance().run()` every 20 ms in `robotPeriodic()`, schedules the auto command in `autonomousInit()`, cancels it in `teleopInit()`, and cancels everything in `testInit()`. Keep logic out of the mode methods. Use commands and triggers instead.
- `RobotContainer.java`: creates subsystems and controllers, binds buttons in `configureBindings()`, and returns the auto in `getAutonomousCommand()` (currently `null`). It also owns the global flags:
  - `isRed` / `isComp` are exposed on the dashboard under `RC`, and passed to the library through `new DemaciaUtils(() -> getIsComp(), () -> getIsRed())`.
  - `setIsComp(true)` calls `LogManager.removeInComp()` once.
- `Constants.java`: numbers only (ports, CAN IDs, gear ratios, setpoints). No logic.

## Conventions for new season code

- One package per subsystem is recommended: `frc/robot/subsystems/<name>/` with `<Name>.java`, `<Name>Constants.java` (configs + IDs) and `<Name>State.java` (state enum).
- Build subsystems on `BaseMechanism` / `StateBaseMechanism` (see `.claude/rules/mechanisms.md` and the `new-mechanism` skill). Build swerve on `Chassis.initialize(new ChassisConfig(...))`.
- Bind buttons only in `RobotContainer.configureBindings()`: `controller.upButton().onTrue(new InstantCommand(() -> arm.setState(ArmState.SCORE)));`. Don't add the mechanism as a requirement for a pure `setState` command, or it will interrupt a running calibration.
- Create objects here and pass them into constructors. Avoid adding new singletons.
- Field and game constants (tag layout, scoring positions) belong here, not in `frc/demacia/vision`.
- `Robot.autonomousInit` uses `Command.schedule()`, which is deprecated for removal in WPILib 2026 (compiler warning). Use `CommandScheduler.getInstance().schedule(cmd)` in new code.
