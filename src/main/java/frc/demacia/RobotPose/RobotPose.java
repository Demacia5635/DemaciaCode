package frc.demacia.RobotPose;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.demacia.RobotPose.Estimation.DemaciaPoseEstimator;
import frc.demacia.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.demacia.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.demacia.RobotPose.Vision.VisionManager;
import frc.demacia.RobotPose.Vision.VisionSource;
import frc.demacia.RobotPose.Vision.VisionTypes.Quest;

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
            Matrix<N3, N1> stateStd, List<VisionSource> sources) {
        this.odometryDataSupplier = odometryDataSupplier;
        this.poseEstimator = new DemaciaPoseEstimator(initialModulePositions, stateStd);
        this.sources = sources;
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

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            poseEstimator.resetPose(new Pose2d(getEstimatedPose().getTranslation(), angle));
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
            SwerveModulePosition[] initialModulePositions, Matrix<N3, N1> stateStd,
            VisionManager visionManager) {
        instance = new RobotPose(odometryDataSupplier, initialModulePositions, stateStd, visionManager.getSources());
    }

    /**
     * Returns the singleton instance, or null if initialize(...) has not been
     * called yet.
     */
    public static synchronized RobotPose getInstance() {
        return instance;
    }
}