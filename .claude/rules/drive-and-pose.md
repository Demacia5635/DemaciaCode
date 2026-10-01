---
paths:
  - "src/main/java/frc/demacia/utils/chassis/**"
  - "src/main/java/frc/demacia/kinematics/**"
  - "src/main/java/frc/demacia/RobotPose/*.java"
  - "src/main/java/frc/demacia/RobotPose/Estimation/**"
  - "src/main/java/frc/demacia/path/**"
---

# Swerve drive, kinematics, pose estimation, path geometry

## Object graph and ownership

- `Chassis` (singleton `SubsystemBase`): `Chassis.initialize(ChassisConfig)` once, then `Chassis.getInstance()` (which is `null` before that). It owns 4 `SwerveModule`s (drive + steer `MotorInterface` + `Cancoder`), a `Pigeon` and a `DemaciaKinematics`. **It no longer creates `RobotPose`**, and its `periodic()` is empty.
- `RobotPose` (singleton, **not** a subsystem): `RobotContainer` creates it with `RobotPose.initialize(Supplier<OdometryData>, moduleLocations, stateStd, VisionConfig)`. **`Robot.robotPeriodic()` calls `RobotPose.getInstance().periodic()` by hand** after the scheduler. If you skip `initialize`, that throws a `NullPointerException`.
- Odometry is passed in as a `Supplier<OdometryData(gyroAngle, modulePositions)>`, so the estimator doesn't depend on `Chassis`. Only `RobotPose.setYaw` reaches back to `Chassis.getInstance()`. Keep it that way.

## `ChassisConfig` / `SwerveModuleConfig`

- `new ChassisConfig(name, SwerveModuleConfig[4], PigeonConfig)` plus `withXxx` limits. `maxDriveVelocity` (5 m/s) and `maxRotationalVelocity` (4 rad/s) are used by `DriveCommand`.
- `new SwerveModuleConfig(name, steerConfig, driveConfig, cancoderConfig).withPosion(Translation2d).withSteerOffset(rad).withMetersFrom360Degs(m)`. The method really is spelled `withPosion`. **Module order: FL, FR, BL, BR.** The steer motor needs `withRadiansMotor` and the drive motor needs `withMeterMotor`.
- `Mk5nConstants` (R1/R2/R3) holds the SDS MK5n gear ratios, wheel diameter and `metersFrom360Degs`.

## Per-loop pose flow (`RobotPose.periodic()`)

1. `poseEstimator.addOdometryData(supplier.get())`
2. `source.periodic()` for every vision source.
3. For each source: if it's a `Quest` that `hasDrifted()`, re-anchor it to the current estimate. Otherwise, if `shouldUpdate()`, add each `TimestampedVisionMeasurement(pose, captureTime, stdDevs)`.

`DemaciaPoseEstimator` keeps 1.5 s (`HISTORY_LENGTH_SECONDS`) of odometry twists. A vision measurement is inserted **at its capture time** (splitting the twist if needed), and the history is replayed. Fusion is per-axis: an std dev of `0` in the state std means vision never changes that axis, and `+∞` in a measurement means that axis is ignored. Use `getEstimatedPose()`, or `getEstimatedPoseAt(t)` for latency-compensated lookups.

## Driving

- `DriveCommand(chassis, controller)`: does nothing in autonomous. Left stick = translation, `rightTrigger − leftTrigger` = rotation. Inputs are squared and multiplied by the max velocity. The direction is +1 when `RobotCommon.getIsRed()` and −1 otherwise. Precision mode ÷4 applies to translation only. It converts with `ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro)` (field → robot), then calls `setSpeedsFieldRel`. `end()` stops the chassis.
- `SwerveModule.setState` optimizes the angle (flips the wheel if the turn is > 90°), holds the steer still if the error is ≤ 0.7°, and drives with 0 power when the target speed is 0.

## Known issues — read before touching drive code

- **Misleading name:** `setSpeedsFieldRel(speeds)` goes straight into `DemaciaKinematics.toSwerveModuleStates`, which ignores heading (`startRobotPosition` is always zero). So it actually expects **robot-relative** speeds. `DriveCommand` works only because it converts first. `setSpeedsRobotRelWithAccel` converts robot→field before calling it, which is backwards. Rename or fix before adding callers.
- `setSpeedsRobotRelWithAccel` does not limit acceleration. The real limiter `DemaciaKinematics.toSwerveModuleStatesWithLimit` is unused.
- `getChassisSpeedsRobotRel()` passes the gyro **angle** where `toChassisSpeeds` expects angular **velocity**. `getChassisSpeedsFieldRel()` does it correctly.
- `path/Leg` and `path/Circle` are not used yet.
- In simulation, `setSpeedsFieldRel` integrates omega into the simulated Pigeon yaw.
