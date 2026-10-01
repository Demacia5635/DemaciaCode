---
paths:
  - "src/main/java/frc/robot/**"
---

# Season code (`frc/robot`)

This is the only place for game-specific code. The library under `frc/demacia` is shared across seasons and may be overwritten by the sync workflow.

## Files and their jobs

- `Main.java`: entry point. Don't touch it.
- `Robot.java` (`TimedRobot`): `robotPeriodic()` runs `CommandScheduler.getInstance().run()` **and then `RobotPose.getInstance().periodic()`**. It schedules the auto in `autonomousInit()`, cancels it in `teleopInit()`, and cancels everything in `testInit()`. Keep logic out of the mode methods.
- `RobotContainer.java`, in order:
  1. `Chassis.initialize(RobotCChassisConstants.CHASSIS_CONFIG)`
  2. `driveCommand = new DriveCommand(Chassis.getInstance(), controller)` (the controller is a static `CommandController(0, kPS5)`)
  3. `RobotPose.initialize(() -> new OdometryData(gyro, modulePositions), moduleLocations, STATE_STD, VisionConstants.visionConfig)`
  4. `configureBindings()` (empty), `setDefaultCommands()` (chassis → `driveCommand`), `setController()` (empty)
  
  `getAutonomousCommand()` returns `null`. It also publishes an empty `"RC"` sendable that `RobotCommon` later replaces. Delete it in new code.
- `chassis/RobotBChassisConstants.java`, `chassis/RobotCChassisConstants.java`: a full `ChassisConfig` per physical robot (CAN IDs, gear ratios, PID, module locations, steer offsets, `STATE_STD`).
- `vision/VisionConstants.java`: camera/Quest configs and the `VisionConfig` passed to `RobotPose`.
- `Constants.java`: numbers only. No logic.

## ⚠️ Which robot?

`RobotContainer` uses **`RobotCChassisConstants`**. In it, the pigeon ID, every PID/FF value and all four steer offsets are `0 // TODO`, so the swerve will not drive correctly. `RobotBChassisConstants` has measured values (Mk5n R2, pigeon 14, real PID). Always confirm which robot is being targeted before changing chassis constants. The vision offsets in `VisionConstants` are also all `0 // TODO`.

## Conventions for new season code

- One package per subsystem: `frc/robot/subsystems/<name>/` with `<Name>.java`, `<Name>Constants.java` (configs + IDs) and `<Name>State.java` (state enum).
- Build subsystems on `BaseMechanism` / `StateBaseMechanism` (see `.claude/rules/mechanisms.md` and the `new-mechanism` skill).
- Bind buttons in `configureBindings()`: `controller.upButton().onTrue(new InstantCommand(() -> arm.setState(ArmState.SCORE)));`. Don't make a pure `setState` command require the mechanism, or it will interrupt a running calibration.
- Create objects here and pass them into constructors. Avoid new singletons.
- Field and game constants (scoring positions, game-piece logic) belong here. AprilTag positions come from WPILib's field layout.
- `Command.schedule()` is deprecated for removal in WPILib 2026 (compiler warning in `Robot.java`). Use `CommandScheduler.getInstance().schedule(cmd)` in new code.
