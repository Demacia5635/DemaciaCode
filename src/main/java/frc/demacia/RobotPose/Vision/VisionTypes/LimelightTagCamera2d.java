package frc.demacia.RobotPose.Vision.VisionTypes;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.RobotPose.Vision.BaseVisionSource;
import frc.demacia.RobotPose.Vision.LimelightHelpers;
import frc.demacia.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera2dConfig;

/**
 * Wraps a single Limelight camera in 2D mode as a VisionSource.
 *
 * 
 *
 */
public class LimelightTagCamera2d extends BaseVisionSource {
    private String limelightName;
    private NetworkTable Table;

    private Pose2d pose;
    private double timestampSeconds;

    private final AprilTagFieldLayout aprilTagFieldLayout;

    private double lastFrameCounterValue = 0;
    private double lastFrameCounterChangeTime = -1;

    private static final double CAMERA_STALE_TIMEOUT_SECONDS = 0.5;

    /**
     * @param config Static configuration for this source. name is the Limelight's
     *               NetworkTables name (empty string "" for the default/only Limelight on a
     *               robot with just one camera).
     * */
    public LimelightTagCamera2d(LimelightTagCamera2dConfig config) {
        super(config);
        limelightName = "limelight-" + config.name;
        Table = NetworkTableInstance.getDefault().getTable(limelightName);
        pose = Pose2d.kZero;
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    /**
     * @return if the camera should send it's Pose estimation to RobotPose.
     */
    @Override
    public boolean shouldUpdate() {
        if (Table == null) {
            return false;
        }
        return Table.getEntry("tv").getDouble(0.0) >= 0.1;
    }

    /**
     * Limelight cameras communicate over NetworkTables; LimelightHelpers does not expose a
     * direct "is this Limelight physically connected" boolean, so connectivity is inferred
     * from LimelightHelpers.getHeartbeat(name) - a counter that increments once per
     * frame while the camera is alive . Since it's a raw counter, not a
     * boolean, connectivity has to be inferred by checking whether it's still CHANGING over
     * time, not just reading it once - so this tracks the last-seen value and when it last
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

    /**
     * 
     *
     * @return A list of pose measurements from this loop, or an empty list if none should
     *         be reported (e.g. no valid tag, or the candidate was rejected by your own
     *         confidence logic).
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        return List.of(new TimestampedVisionMeasurement(pose, timestampSeconds, std));
    }

   
    @Override
    public void periodic() {
        if (shouldUpdate()) {
            updatePose();
        }
    }

    private Pose2d updatePose() {
        double latency = (Table.getEntry("tl").getDouble(0.0) + Table.getEntry("cl").getDouble(0.0))/1000.0;
        timestampSeconds = Timer.getFPGATimestamp() - latency;
        // Heading at the frame's capture time, not now (the robot may turn during the latency).
        Rotation2d heading = RobotPose.getInstance().getEstimatedPoseAt(timestampSeconds).getRotation();

        pose = new Pose2d((getTag().toTranslation2d()).minus(getRobotToTag(heading)), heading);
        return pose;
    }

    private Translation2d getRobotToTag(Rotation2d heading) {
        return getCameraToTag().plus(
            offset.getTranslation().toTranslation2d().rotateBy(heading));
    }

    private Translation2d getCameraToTag() {
        return new Translation2d(getDistanceFromCamera(),
            new Rotation2d(Math.toRadians(-Table.getEntry("tx").getDouble(0.0)) + offset.getRotation().getZ()));
    }

    private double getDistanceFromCamera() {
        double deltaHeight = getTag().getZ() - offset.getZ();
        double alpha = offset.getRotation().getY() + Math.toRadians(Table.getEntry("ty").getDouble(0.0));
        double distance = Math.abs(deltaHeight / Math.tan(alpha)) / Math.cos(Math.toRadians(Table.getEntry("tx").getDouble(0.0)));

        return distance;   
    }

    private Translation3d getTag() {
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose((int) Table.getEntry("tid").getDouble(0.0));
        if (tagPose.isEmpty()) {
            return new Translation3d();
        }
        return tagPose.get().getTranslation();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is see", () -> shouldUpdate(), null);
    }
}