---
paths:
  - "src/main/java/frc/demacia/RobotPose/Vision/**"
  - "src/main/java/frc/robot/vision/**"
---

# Vision sources (`frc/demacia/RobotPose/Vision`)

Every vision source turns camera/headset data into `TimestampedVisionMeasurement(pose, captureTimeSeconds, stdDevs)` for `RobotPose`. The classes already have good Javadoc explaining the math, so read it before changing anything.

## Structure

- `VisionSource` (interface): `getName()`, `periodic()`, `shouldUpdate()`, `getPoseEstimates()`, `isConnected()`.
- `BaseVisionSource` (abstract, `Sendable`): stores `name`, `offset` (`Transform3d` robot→camera), and `std`. It adds `vision/<name>` and `vision/<name>/field` to the dashboard and registers with `ElasticGenerator`.
- Implementations in `VisionTypes/`:
  - `LimelightTagCamera2d`: computes x/y from `tx`/`ty`/`tid` + tag height. The tag positions come from WPILib `AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)`. The heading comes from the estimate at capture time.
  - `LimelightTagCamera3d`: MegaTag2. It sends the camera offset to the Limelight once (this overwrites the web-UI setting) and the robot heading every loop, then reads `getBotPoseEstimate_wpiBlue_MegaTag2`.
  - `Quest`: QuestNav. It must be **anchored** with `setPose` before use, and `RobotPose` re-anchors it when `hasDrifted()`. Frames are trusted only after a settle time following each reset.
- All three send **heading std = +∞**: vision corrects x/y only, and the heading comes from the gyro.
- `isConnected()` uses the Limelight heartbeat (stale after 0.5 s) or the QuestNav connection.

## Configuring sources (season side)

Configs live in `frc/robot/vision/VisionConstants.java`:

```java
new LimelightTagCamera2dConfig(name, robotToCamera /*Transform3d*/, std /*x, y, theta*/)
new LimelightTagCamera3dConfig(name, robotToCamera, std)
new QuestConfig(name, robotToHeadset, std)
VisionConfig visionConfig = new VisionConfig().addSource(cfg1).addSource(cfg2) ...;
```

- The NetworkTables table / Limelight hostname is `"limelight-" + name`. The Limelight's hostname must match.
- A config's `visionSourceType` picks the class (`BaseVisionSourceConfig.VisionSourceType` factory). To add a new kind of source: create a config subclass, an implementation of `BaseVisionSource`, and an enum entry.
- ⚠️ Every offset in `VisionConstants` is currently `0, 0, 0 // TODO`. Until they're measured, vision poses are wrong by the camera's offset.

## Rules

- Sources may read `RobotPose.getInstance()` for the heading at capture time (same package). They must not touch `Chassis`.
- Always report the **capture** timestamp, not the arrival time. The estimator replays history using it.
- `LimelightHelpers` is the official third-party file. Don't edit it. Replace it with a newer upstream version.
