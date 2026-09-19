package frc.demacia.RobotPose.Vision.VisionTypes;

import java.util.List;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.RobotPose.Vision.BaseVisionSource;
import frc.demacia.RobotPose.Vision.LimelightHelpers;
import frc.demacia.RobotPose.Vision.LimelightHelpers.PoseEstimate;
import frc.demacia.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;

/**
 * Wraps a single Limelight camera in 3D (MegaTag2) mode as a VisionSource. 
 *
 */
public class LimelightTagCamera3d extends BaseVisionSource {
    private String limelightName;

    private PoseEstimate pose;

    private double lastFrameCounterValue = 0;
    private double lastFrameCounterChangeTime = -1;

    private static final double CAMERA_STALE_TIMEOUT_SECONDS = 0.5;

    /**
     * @param config              Static configuration for this source. name is the
     *                            Limelight's NetworkTables name. offset is currently unused
     *                            by this class (Limelight's own pose math accounts for
     *                            camera placement via its own calibration) but is accepted
     *                            here as part of the shared VisionSourceConfig shape used
     *                            by every VisionSource.
     */
    public LimelightTagCamera3d(LimelightTagCamera3dConfig config) {
        super(config);
        limelightName = "limelight-" + config.name;
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, 
            offset.getX(), 
            offset.getY(), 
            offset.getZ(), 
            Math.toDegrees(offset.getRotation().getX()), 
            Math.toDegrees(offset.getRotation().getY()), 
            Math.toDegrees(offset.getRotation().getZ()));
    }

    /**
     * Gated on LimelightHelpers.getTV(), matching the tv >= 0.1 pattern from the user's own
     * old TagPose.isSeeTag() (explicitly confirmed as the model for this method) --
     * getTV() is LimelightHelpers' own boolean wrapper around that same NetworkTables
     * entry, so this is the same gate, just via the standard helper rather than a raw read.
     */
    @Override
    public boolean shouldUpdate() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Limelight cameras communicate over NetworkTables; LimelightHelpers does not expose a
     * direct "is this Limelight physically connected" boolean, so connectivity is inferred
     * from LimelightHelpers.getHeartbeat(name) -- a counter that increments once per
     * frame while the camera is alive (confirmed real API). Since it's a raw counter, not a
     * boolean, connectivity has to be inferred by checking whether it's still CHANGING over
     * time, not just reading it once -- so this tracks the last-seen value and when it last
     * changed, and reports disconnected only if the counter has been stuck for longer than
     * CAMERA_STALE_TIMEOUT_SECONDS.
     */
    @Override
    public boolean isConnected() {
        double currentFrameCounter = LimelightHelpers.getHeartbeat(limelightName);
        double now = Timer.getFPGATimestamp();

        if (currentFrameCounter != lastFrameCounterValue) {
            lastFrameCounterValue = currentFrameCounter;
            lastFrameCounterChangeTime = now;
        }

        return (now - lastFrameCounterChangeTime) < CAMERA_STALE_TIMEOUT_SECONDS;
    }

    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        return List.of(new TimestampedVisionMeasurement(pose.pose, pose.timestampSeconds, std));
    }

    /**
     * Pushes the current gyro heading via SetRobotOrientation() BEFORE this loop's pose can
     * be correctly read back -- confirmed hard MegaTag2 requirement, not optional. This is
     * exactly why RobotPose calls periodic() on every VisionSource before calling
     * shouldUpdate()/getPoseEstimates() on any of them (the two-pass design): this call
     * must happen before getPoseEstimates() runs for THIS SAME LOOP, and the two-pass
     * structure guarantees that ordering regardless of source iteration order.
     */
    @Override
    public void periodic() {
        Rotation2d heading = RobotPose.getInstance().getGyroAngle();
        LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    
        pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> shouldUpdate(), null);
    }
}