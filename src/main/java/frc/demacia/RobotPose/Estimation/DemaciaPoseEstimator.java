package frc.demacia.RobotPose.Estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/**
 * Pose estimator using the twist-sequence + splice + full-replay mechanism
 * (matching the approach used by Mechanical-Advantage's (6328) PoseEstimator.java because WPILib's class has an acknowledged limitation
 * where multiple vision corrections do not compound - each corrects against raw odometry
 * rather than the previously-corrected estimate). 
 */
public class DemaciaPoseEstimator {

    /**
     * How long a twist/vision-correction entry is kept in the active update map before
     * being folded into basePose.
     */
    public static final double HISTORY_LENGTH_SECONDS = 1.5;

    private final DemaciaOdometry odometry;

    /** Per-axis (x meters, y meters, theta radians) squared state std devs, used as the "q" term in the per-axis gain formula below. */
    private final double[] stateVarianceByAxis = new double[3];

    private Pose2d initialPose = new Pose2d();
    private Pose2d latestPose = new Pose2d();
    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();

    public DemaciaPoseEstimator(SwerveModulePosition[] initialPositions, Translation2d[] moduleLocations,
            Matrix<N3, N1> stateSTD) {
        this.odometry = new DemaciaOdometry(initialPositions, moduleLocations);
        setStateStd(stateSTD);
        this.latestPose = this.initialPose;
    }

    public final void setStateStd(Matrix<N3, N1> stateSTD) {
        for (int i = 0; i < 3; ++i) {
            stateVarianceByAxis[i] = stateSTD.get(i, 0) * stateSTD.get(i, 0);
        }
    }

    public void addOdometryData(OdometryData odometryData) {
        double timestamp = Timer.getFPGATimestamp();
        Twist2d twist = odometry.updateOdometry(odometryData.gyroAngle(), odometryData.swerveModules());
        updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
        update();
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        // A NaN once fused is replayed every loop and then folded into the base pose, so it would never go away.
        if (!isValidMeasurement(visionRobotPose, stdDevs)) {
            return;
        }

        VisionUpdate visionUpdate = new VisionUpdate(visionRobotPose, stdDevs);

        if (updates.containsKey(timestampSeconds)) {
            updates.get(timestampSeconds).visionUpdates.add(visionUpdate);
            update();
            return;
        }

        var prevEntry = updates.floorEntry(timestampSeconds);
        var nextEntry = updates.ceilingEntry(timestampSeconds);

        if (prevEntry == null || nextEntry == null) {
            // No bracketing "next" entry (or no "prev" entry, e.g. timestamp predates
            // everything currently tracked) - silently drop, per explicit decision.
            return;
        }

        double prevKey = prevEntry.getKey();
        double nextKey = nextEntry.getKey();
        PoseUpdate nextUpdate = nextEntry.getValue();

        double frac0 = (timestampSeconds - prevKey) / (nextKey - prevKey);
        double frac1 = (nextKey - timestampSeconds) / (nextKey - prevKey);

        Twist2d twist0 = new Twist2d(nextUpdate.twist.dx * frac0, nextUpdate.twist.dy * frac0,
                nextUpdate.twist.dtheta * frac0);
        Twist2d twist1 = new Twist2d(nextUpdate.twist.dx * frac1, nextUpdate.twist.dy * frac1,
                nextUpdate.twist.dtheta * frac1);

        List<VisionUpdate> newEntryVisionUpdates = new ArrayList<>();
        newEntryVisionUpdates.add(visionUpdate);

        updates.put(timestampSeconds, new PoseUpdate(twist0, newEntryVisionUpdates));
        updates.put(nextKey, new PoseUpdate(twist1, nextUpdate.visionUpdates));

        update();
    }

    /** Pose must be finite; std devs must be non-negative and not NaN (infinite means "not measured"). */
    private static boolean isValidMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs) {
        if (!Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
                || !Double.isFinite(pose.getRotation().getRadians())) {
            return false;
        }
        for (int i = 0; i < 3; i++) {
            double std = stdDevs.get(i, 0);
            if (Double.isNaN(std) || std < 0) {
                return false;
            }
        }
        return true;
    }

    private void update() {
        double now = Timer.getFPGATimestamp();

        while (updates.size() > 1 && updates.firstKey() < now - HISTORY_LENGTH_SECONDS) {
            var oldestEntry = updates.pollFirstEntry();
            initialPose = oldestEntry.getValue().apply(initialPose, stateVarianceByAxis);
        }

        Pose2d pose = initialPose;
        for (PoseUpdate poseUpdate : updates.values()) {
            pose = poseUpdate.apply(pose, stateVarianceByAxis);
        }
        latestPose = pose;
    }

    public Pose2d getEstimatedPose() {
        return latestPose;
    }

    /**
     * @param pose      The new field pose.
     * @param gyroAngle The raw gyro reading that corresponds to pose's heading.
     */
    public void resetPose(Pose2d pose, Rotation2d gyroAngle) {
        odometry.resetPose(pose, gyroAngle);
        updates.clear();
        initialPose = pose;
        latestPose = pose;
    }


    public record OdometryData(Rotation2d gyroAngle, SwerveModulePosition[] swerveModules) {
    }

    private static final class VisionUpdate {
        private final Pose2d pose;
        private final Matrix<N3, N1> stdDevs;
        private VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.stdDevs = stdDevs;
        }
    }

    private static final class PoseUpdate {
        private final Twist2d twist;
        private final List<VisionUpdate> visionUpdates;
        private PoseUpdate(Twist2d twist, List<VisionUpdate> visionUpdates) {
            this.twist = twist;
            this.visionUpdates = visionUpdates;
        }

        private Pose2d apply(Pose2d lastPose, double[] stateVarianceByAxis) {
            Pose2d pose = lastPose.exp(twist);
            if (visionUpdates.isEmpty()) {
                return pose;
            }

            // Measurements with the same timestamp are fused (inverse-variance) into one before
            // the gain is applied, so N measurements of variance r count as one of variance r/N.
            double[] residualByAxis = new double[3];
            double[] measurementVarianceByAxis = new double[3];
            for (int axis = 0; axis < 3; axis++) {
                double informationSum = 0;
                double weightedResidualSum = 0;
                int exactCount = 0;
                double exactResidualSum = 0;
                for (VisionUpdate visionUpdate : visionUpdates) {
                    double std = visionUpdate.stdDevs.get(axis, 0);
                    double variance = std * std;
                    double residual = residual(axis, visionUpdate.pose, pose);
                    if (variance == 0.0) {
                        exactCount++;
                        exactResidualSum += residual;
                    } else {
                        informationSum += 1.0 / variance;
                        weightedResidualSum += residual / variance;
                    }
                }
                if (exactCount > 0) {
                    residualByAxis[axis] = exactResidualSum / exactCount;
                    measurementVarianceByAxis[axis] = 0.0;
                } else {
                    residualByAxis[axis] = weightedResidualSum / informationSum;
                    measurementVarianceByAxis[axis] = 1.0 / informationSum;
                }
            }

            double[] kalmanGainByAxis = new double[3];
            for (int row = 0; row < 3; row++) {
                if (stateVarianceByAxis[row] == 0.0) {
                    kalmanGainByAxis[row] = 0.0;
                } else {
                    kalmanGainByAxis[row] = stateVarianceByAxis[row]
                            / (stateVarianceByAxis[row] + Math.sqrt(stateVarianceByAxis[row] * measurementVarianceByAxis[row]));
                }
            }

            // The gain is applied per field axis (the frame the std devs are given in), not to
            // pose.log(vision), whose dx/dy are robot-relative and bent by dtheta.
            return new Pose2d(
                    pose.getX() + kalmanGainByAxis[0] * residualByAxis[0],
                    pose.getY() + kalmanGainByAxis[1] * residualByAxis[1],
                    pose.getRotation().plus(new Rotation2d(kalmanGainByAxis[2] * residualByAxis[2])));
        }

        /** Field-frame residual of one axis (x, y in meters; theta wrapped to [-pi, pi]). */
        private static double residual(int axis, Pose2d measured, Pose2d estimate) {
            switch (axis) {
                case 0:
                    return measured.getX() - estimate.getX();
                case 1:
                    return measured.getY() - estimate.getY();
                default:
                    return measured.getRotation().minus(estimate.getRotation()).getRadians();
            }
        }
    }
}