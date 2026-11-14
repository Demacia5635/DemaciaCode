package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.demacia.path.calculator.ArcCalculator;
import frc.demacia.path.calculator.LegCalculator;
import frc.demacia.path.constants.PathConstants;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.SegmentBase;
import frc.demacia.path.trapezoid.DemaciaTrapezoid;
import frc.demacia.utils.log.Log;

public class DemaciaTrajectory {

    private List<SegmentBase> segments;
    private DemaciaTrapezoid trapezoid;
    private ProfiledPIDController headingController;

    private int currentSegmentIndex;
    private boolean isFinishedTrajectory;
    private boolean resetHeadingController;

    public DemaciaTrajectory(List<PathPoint> points) {
        this.segments = new ArrayList<SegmentBase>();
        this.trapezoid = new DemaciaTrapezoid(PathConstants.MAX_VELOCITY, PathConstants.MAX_ACCELERATION);
        this.headingController = new ProfiledPIDController(PathConstants.HEADING_KP, 0, 0,
                new Constraints(PathConstants.MAX_ANGULAR_VELOCITY, PathConstants.MAX_ANGULAR_ACCELERATION));
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.resetHeadingController = true;

        if (points.size() < 2) {
            Log.log("Not enough points to build a trajectory");
            this.isFinishedTrajectory = true;
            return;
        }

        buildSegments(points);
        limitEndVelocities();

        for (int i = 0; i < segments.size(); i++) {
            Log.log("segments.get(" + i + "): " + segments.get(i));
        }
    }

    public void startTrajectory() {
        isFinishedTrajectory = segments.isEmpty();
        currentSegmentIndex = 0;
        resetHeadingController = true;
    }

    public boolean getIsFinishedTrajectory() {
        return isFinishedTrajectory;
    }

    private void buildSegments(List<PathPoint> points) {
        Translation2d currentStart = points.get(0).getTranslation();

        for (int i = 1; i < points.size() - 1; i++) {
            PathPoint point = points.get(i);
            Translation2d prev = points.get(i - 1).getTranslation();
            Translation2d here = point.getTranslation();
            Translation2d next = points.get(i + 1).getTranslation();
            double v = point.getVelocity();

            double turn = Math.abs(MathUtil.angleModulus(
                    next.minus(here).getAngle().getRadians() - here.minus(prev).getAngle().getRadians()));
            double maxR = Math.min(prev.getDistance(here), here.getDistance(next)) * PathConstants.MAX_RADIUS_TO_LEG_RATIO;
            boolean reversal = turn > Math.PI - PathConstants.MIN_TURN_FOR_ARC;

            if (v == 0 || reversal || turn < PathConstants.MIN_TURN_FOR_ARC || maxR < 1e-6) {
                segments.add(new LineSegment(currentStart, point, reversal ? 0 : v, PathConstants.MAX_VELOCITY));
                currentStart = here;
                continue;
            }

            double r = v * v / PathConstants.MAX_LATERAL_ACCELERATION;
            if (r > maxR) {
                r = maxR;
                v = Math.sqrt(PathConstants.MAX_LATERAL_ACCELERATION * r);
            }
            ArcSegment corner = ArcCalculator.calculateCenter(prev, here, next, r);
            List<Pose2d> arcPoints = LegCalculator.returnPoint(prev, here, corner, r, point.getRotation());

            segments.add(new LineSegment(currentStart, arcPoints.get(0), v, PathConstants.MAX_VELOCITY));
            segments.add(new ArcSegment(arcPoints.get(0).getTranslation(), arcPoints.get(1),
                    corner.getCenter(), corner.getIsLeft(), r, v, v));
            currentStart = arcPoints.get(1).getTranslation();
        }
        segments.add(new LineSegment(currentStart, points.get(points.size() - 1), 0, PathConstants.MAX_VELOCITY));
    }

    // lower each end velocity so the next segment can still slow down to its own end velocity
    private void limitEndVelocities() {
        for (int i = segments.size() - 2; i >= 0; i--) {
            SegmentBase segment = segments.get(i);
            SegmentBase next = segments.get(i + 1);
            double reachable = Math.sqrt(next.getEndVelocity() * next.getEndVelocity()
                    + 2 * PathConstants.MAX_ACCELERATION * next.getLength());
            segment.setEndVelocity(Math.min(segment.getEndVelocity(), Math.min(next.getMaxVelocity(), reachable)));
        }
    }

    public ChassisSpeeds calculateSpeeds(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        if (isFinishedTrajectory) {
            return new ChassisSpeeds();
        }
        double heading = currentPose.getRotation().getRadians();
        if (resetHeadingController) {
            headingController.reset(heading, currentSpeeds.omegaRadiansPerSecond);
            resetHeadingController = false;
        }

        SegmentBase segment = segments.get(currentSegmentIndex);
        Translation2d position = currentPose.getTranslation();
        double currentVelocity = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        double speed = Math.min(trapezoid.nextVelocity(segment.getDistanceLeft(position), currentVelocity,
                segment.getEndVelocity(), segment.getMaxVelocity()), segment.getMaxVelocity());
        Translation2d velocity = new Translation2d(speed, segment.getDirection(position, currentVelocity));

        double omega = headingController.calculate(heading, segment.getEndPose().getRotation().getRadians())
                + headingController.getSetpoint().velocity;
        ChassisSpeeds speeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);

        if (!segment.isFinished(position)) {
            return speeds;
        }
        // a stop point is done only once the robot really stopped
        if (segment.getEndVelocity() == 0 && currentVelocity > PathConstants.MAX_VELOCITY_TO_CONSIDER_STOPPED) {
            return new ChassisSpeeds(0, 0, omega);
        }
        if (currentSegmentIndex < segments.size() - 1) {
            currentSegmentIndex++;
            return speeds;
        }
        if (Math.abs(currentPose.getRotation().minus(segment.getEndPose().getRotation()).getRadians()) < PathConstants.HEADING_TOLERANCE) {
            isFinishedTrajectory = true;
            return new ChassisSpeeds();
        }
        return new ChassisSpeeds(0, 0, omega);
    }
}
