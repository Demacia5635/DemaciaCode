---
paths:
  - "src/main/java/frc/demacia/utils/chassis/**"
  - "src/main/java/frc/demacia/kinematics/**"
  - "src/main/java/frc/demacia/odometry/**"
  - "src/main/java/frc/demacia/path/**"
---

# Swerve drive, kinematics, odometry, path geometry

## Object graph and ownership

- `Chassis` (singleton, `SubsystemBase`) is created **once** with `Chassis.initialize(ChassisConfig)` and read with `Chassis.getInstance()`. `getInstance()` returns `null` before `initialize`.
- `Chassis` owns 4 `SwerveModule`s (each: drive `MotorInterface` + steer `MotorInterface` + `Cancoder`), a `Pigeon` gyro and a `DemaciaKinematics`. Its constructor also calls `RobotPose.initialize(...)`.
- `RobotPose` (singleton) owns `DemaciaPoseEstimator` (which owns `DemaciaOdometry`), a `Vision` (Limelight tags) and a `Quest`.
- **Circular dependency:** `DemaciaPoseEstimator.getEstimatedPose()`, `Vision` and `TagPose` call `Chassis.getInstance()` to read the gyro. Creation order therefore matters. In new code, pass the gyro angle in as a parameter or `Supplier<Rotation2d>` instead.

## Per-loop flow (`Chassis.periodic()`)

`OdometryObservation(timestamp, gyroAngle, modulePositions)` → `RobotPose.update()`:
1. `vision.updateValues()`
2. Add odometry, but only while the robot is level (`|accel.x| < 0.3 && |accel.z| < 0.3`; this skips updates while tipping or bumping).
3. If the Quest is connected and its initial pose was set → `addQuestMeasurement`.
4. If a tag is seen → `addVisionMeasurement`. Timestamp = now − 0.05 s (a fixed latency, not the camera's real latency).

`getEstimatedPose()` returns x/y from the fusion, but the **rotation always comes from the gyro**. The estimator keeps a 1.5 s odometry buffer, so vision is applied at the past timestamp and then replayed.

## Driving

- `DriveCommand(chassis, controller)`: left stick = translation, `leftTrigger − rightTrigger` = rotation, inputs squared, × max velocity. The direction flips with `DemaciaUtils.getIsRed()`. `invertPrecisionMode()` divides by 4.
- `Chassis.followTrajectory(SwerveSample)` follows a Choreo sample: feed-forward speeds + PID on x, y and heading.
- `SwerveModule.setState` optimizes the angle itself (flips the wheel and reverses speed when the turn is > 90°) and compensates for steer→drive coupling with `SteerVelToDriveVel`. Configure steer motors with `withRadiansMotor` and drive motors with `withMeterMotor`.

## Known broken — fix before relying on the chassis

- **Module positions are never set.** In the `Chassis` constructor, `modulePositions[i] = ...` is commented out, and `SwerveModuleConfig.position` is a single `double` instead of a `Translation2d`. `DemaciaKinematics`, `DemaciaOdometry` and `RobotPose` all receive an array of `null`s, which will throw a `NullPointerException` on use.
- **Frame confusion.** `DemaciaKinematics.toSwerveModuleStates` ignores robot heading (`startRobotPosition` is always zero), so it treats its input as **robot-relative**. `setVelocities` documents its input as **field-relative**. `setRobotRelSpeedsWithAccel` converts robot→field and then calls `setVelocities`. In `DriveCommand`, normal mode goes through that conversion but precision mode does not. Choose one convention, document it on every method, and test on the robot.
- `setRobotRelSpeedsWithAccel` does **not** limit acceleration. `toSwerveModuleStatesWithLimit` (the real limiter, configured by `KinematicsConstants`) exists but nothing calls it.
- `Chassis.periodic()` has `updateCommon()` commented out (`TODO: RETORN IT`), and there is season code (`isRotateToHub`, turret) mixed in. Move game-specific behavior to `frc/robot`.
- `path/Leg` and `path/Circle` (tangent line between two circles) are not used anywhere yet.

## Constants

`KinematicsConstants` holds `MAX_ALLOWED_MODULE_VELOCITY` (4 m/s; module speeds are scaled down to fit), cycle time 0.02 s and acceleration limits. `ChassisConfig` holds per-robot limits (`maxLinearAccel`, `maxOmegaVelocity`, `maxRadialAccel`).
