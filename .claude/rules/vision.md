---
paths:
  - "src/main/java/frc/demacia/vision/**"
---

# Vision (`frc/demacia/vision`)

## What is used, and what isn't

- **Used:** `vision/utils/Vision` combines several `vision/TagPose` objects (one per Limelight). `RobotPose` creates it as `new Vision(VisionConstants.Tags.TAGS_ARRAY)`. `vision/subsystem/Quest` (QuestNav headset) is also created by `RobotPose`.
- **Not used anywhere:** `vision/subsystem/Tag` and `vision/subsystem/ObjectPose`, both subsystems that nothing constructs. `vision/ObjectPose` duplicates the name of `vision/subsystem/ObjectPose`. Before editing one of these, confirm which class is actually used.
- `vision/utils/LimelightHelpers` is the official Limelight helper file (~1,600 lines, third-party). Don't modify it. Replace it with a newer upstream version instead.

## Cameras

- `new Camera(name, robotToCamTranslation3d, pitchDeg, yawDeg, isCropping, isObjectCamera)`. The NetworkTables table is `"limelight-" + name`, so the Limelight's hostname must match.
- Each `TagPose` adds dashboard buttons: `setTo3d` / `setTo2d`, and `chassis/reset gyro by camera <name>`. That button switches to pipeline 5, sets the chassis yaw from the tag, then switches back to pipeline 0.
- **`VisionConstants.Tags.TAGS_ARRAY` is empty** because every camera entry is commented out. Until a camera is added there, `Vision.isSeeTag()` is always false and pose estimation runs on odometry (+ Quest) only.

## Fusion details

- `Vision.getPoseEstimation()` is a confidence-weighted average of all cameras' x/y. The angle comes from the gyro (`Chassis.getInstance()`, which is a layer violation).
- Confidence drops with distance (`BEST_RELIABLE_DISTANCE` 1 m → `WORST_RELIABLE_DISTANCE` 4 m) and with robot speed (1 → 3 m/s).
- `PREDICT_X/Y/OMEGA` shift the crop window using chassis speed (`TagPose` reads `Chassis.getInstance()`).
- Quest: `RobotPose.setQuestPose(pose)` must be called once (usually from a vision pose) before its measurements are used. If it disconnects, this is only logged (`TODO: change to led signal`).

## Season data inside the library — move out in the new version

`VisionConstants` contains this season's field: tag heights (hub, outpost, depot), `O_TO_TAG` positions, and `Vision.isTagHub(...)`. Its comments say "2026 REEFSCAPE" but the names are hub/outpost/depot. In new code, load tag positions from WPILib's `AprilTagFieldLayout` or keep them in `frc/robot`. Pass the gyro angle and chassis speeds in as `Supplier`s, the way the unused `vision/subsystem/Tag` constructor already does.
