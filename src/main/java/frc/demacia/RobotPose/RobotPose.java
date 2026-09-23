package frc.demacia.RobotPose;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.RobotPose.Estimation.DemaciaPoseEstimator;
import frc.demacia.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.demacia.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.demacia.RobotPose.Vision.VisionConfig;
import frc.demacia.RobotPose.Vision.VisionSource;
import frc.demacia.RobotPose.Vision.VisionTypes.Quest;
import frc.demacia.utils.chassis.Chassis;

/**
 * Top-level per-loop coordinator. Owns a DemaciaPoseEstimator (constructed
 * internally, per
 * confirmed design) and drives it every loop from a caller-supplied odometry
 * source plus
 * whatever VisionSources were configured via VisionManager.
 *
 * RobotPose does NOT hold a reference to VisionManager itself -- it receives
 * the already-
 * unpacked List<VisionSource> and Optional<Quest> that VisionManager produced
 * (confirmed:
 * "it shouldn't depend, just get the lists"). This keeps RobotPose's own
 * dependencies
 * narrow and makes it testable without ever touching VisionManager's
 * singleton/builder
 * machinery.
 *
 * The ONE deliberate exception to "RobotPose never knows which concrete
 * VisionSource it's
 * talking to" is the Optional<Quest> re-anchor check in periodic() --
 * everything else about
 * source handling is fully generic across the List<VisionSource>.
 *
 * Singleton, matching VisionManager's own static-factory pattern (private
 * constructor,
 * static initialize/getInstance) and the old reference RobotPose.java's
 * established shape.
 */
public final class RobotPose {

    private static RobotPose instance;

    private final DemaciaPoseEstimator poseEstimator;
    private final Supplier<OdometryData> odometryDataSupplier;
    private final List<VisionSource> sources;

    private RobotPose(Supplier<OdometryData> odometryDataSupplier, SwerveModulePosition[] initialModulePositions,
            Translation2d[] moduleLocations, Matrix<N3, N1> stateStd, List<VisionSource> sources) {
        this.odometryDataSupplier = odometryDataSupplier;
        this.poseEstimator = new DemaciaPoseEstimator(initialModulePositions, moduleLocations, stateStd);
        this.sources = sources;

        addLog();
    }

    private void addLog() {
        SmartDashboard.putData("chassis/reset gyro",
                new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset gyro 180",
                new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
    }

    /**
     * Runs one loop's worth of pose estimation: odometry first (unconditional),
     * then a
     * two-pass sweep over every VisionSource (periodic() on all of them, THEN
     * shouldUpdate()/getPoseEstimates() on all of them
     */
    public void periodic() {
        poseEstimator.addOdometryData(odometryDataSupplier.get());

        for (VisionSource source : sources) {
            source.periodic();
        }

        for (VisionSource source : sources) {
            if (source instanceof Quest && ((Quest) source).hasDrifted()) {
                ((Quest) source).setPose(poseEstimator.getEstimatedPose());
            }
            else if (source.shouldUpdate()) {
                for (TimestampedVisionMeasurement measurement : source.getPoseEstimates()) {
                    poseEstimator.addVisionMeasurement(measurement.pose(), measurement.timestampSeconds(),
                            measurement.stdDevs());
                }
            }
        }
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPose();
    }

    /** The estimated pose at a past FPGA timestamp (e.g. a vision frame's capture time). */
    public Pose2d getEstimatedPoseAt(double timestampSeconds) {
        return poseEstimator.getPoseAt(timestampSeconds);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose, getGyroAngle());
    }

    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            Chassis.getInstance().setYaw(angle);
            // The gyro was just set to `angle`, so pass it as the gyro reading (the new value may
            // not have been read back from the device yet).
            poseEstimator.resetPose(new Pose2d(getEstimatedPose().getTranslation(), angle), angle);
        }
    }

    public Rotation2d getGyroAngle() {
        return odometryDataSupplier.get().gyroAngle();
    }


    /**
     * Installs the singleton instance, building the internal DemaciaPoseEstimator
     * and
     * wiring up whatever VisionManager already produced. Call this from
     * Robot.robotInit()
     * (or equivalent) AFTER building and initializing VisionManager -- per the
     * confirmed
     * design, RobotPose.initialize(...) is the single entry point; it is what pulls
     * VisionManager's finished getSources()/getQuest() into RobotPose, not the
     * other way
     * around. A second call silently overwrites the previous instance -- the caller
     * is
     * trusted to only call this once, per the user's own confirmed choice to
     * guarantee
     * correct call order themselves rather than have this enforced here.
     */
    public static synchronized void initialize(Supplier<OdometryData> odometryDataSupplier,
            SwerveModulePosition[] initialModulePositions, Translation2d[] moduleLocations,
            Matrix<N3, N1> stateStd, VisionConfig visionManager) {
        instance = new RobotPose(odometryDataSupplier, 
            initialModulePositions, 
            moduleLocations, 
stateStd, 
            visionManager.getSources());
    }

    /**
     * Returns the singleton instance, or null if initialize(...) has not been
     * called yet.
     */
    public static synchronized RobotPose getInstance() {
        return instance;
    }
}