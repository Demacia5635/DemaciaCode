package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.ArrayList;
import java.util.List;

/**
 * DemaciaTrajectory — receives a list of waypoints, builds a sequence of
 * straight legs and arcs, then computes the required ChassisSpeeds every cycle.
 *
 * Build phase  : call build() once before the command starts.
 * Tracking phase: call calculateSpeeds() every execute() cycle.
 */
public class DemaciaTrajectory {

    private static final double CYCLE_TIME         = 0.02;              // seconds per cycle
    private static final double ARRIVE_THRESHOLD   = 0.05;              // meters — switch leg when closer than this
    private static final double ANGLE_ERR_MAX      = Math.toRadians(45);// radians — switch leg if error exceeds this
    private static final double ARC_DONE_THRESHOLD = Math.toRadians(3); // radians — arc is done when heading is this close
    private static final double KP_HEADING         = 3.0;               // P-gain for robot rotation correction

    private final List<Leg> legs = new ArrayList<>();
    private int currentLegIndex = 0;
    private boolean finished = false;

    // =========================================================================
    //  PATH BUILDING
    // =========================================================================

    /**
     * Builds the full leg list from the given waypoints.
     * Step A: compute circle centers for every internal waypoint.
     * Step B: compute tangent points and create straight + arc legs.
     *
     * @param points list of at least 2 waypoints
     * @param radius uniform turn radius for the entire path (meters)
     */
    public void build(List<TrajectoryPoint> points, double radius) {
        legs.clear();
        currentLegIndex = 0;
        finished = false;

        if (points.size() < 2) return;

        // Step A: compute circle centers for internal waypoints (not first or last)
        Circle[] circles = new Circle[points.size()];
        for (int i = 1; i < points.size() - 1; i++) {
            circles[i] = Circle.calculateCircleCenter(
                    points.get(i - 1).pose.getTranslation(),
                    points.get(i + 1).pose.getTranslation(),
                    points.get(i).pose.getTranslation(),
                    radius);
        }

        // Step B.1: straight leg from start to the first tangent point
        Translation2d firstTangent = TangentCalculator.pointToCircleTangent(
                points.get(0).pose.getTranslation(),
                circles[1].center,
                radius,
                circles[1].isLeftTurn);

        addStraightLeg(points.get(0).pose,
                       toPose(firstTangent, points.get(0).pose.getRotation()),
                       points.get(1));

        // Step B.2: arc + straight for each internal waypoint
        for (int i = 1; i < points.size() - 1; i++) {
            Circle curr = circles[i];
            boolean hasNext = (i + 1 < points.size() - 1);

            if (hasNext) {
                Circle next = circles[i + 1];
                Translation2d[] tangents;

                // Same turn direction -> parallel (straight) tangent
                // Opposite turn direction -> cross tangent
                if (curr.isLeftTurn == next.isLeftTurn) {
                    tangents = TangentCalculator.sameTurnTangents(
                            curr.center, next.center, radius, curr.isLeftTurn);
                } else {
                    tangents = TangentCalculator.oppositeTurnTangents(
                            curr.center, next.center, radius, curr.isLeftTurn);
                }

                // Arc from the end of the previous straight to the first tangent point
                addArcLeg(legs.get(legs.size() - 1).end,
                          toPose(tangents[0], angleToTarget(curr.center, tangents[0], curr.isLeftTurn)),
                          curr, points.get(i));

                // Straight leg between the two tangent points
                addStraightLeg(legs.get(legs.size() - 1).end,
                               toPose(tangents[1], angleToTarget(next.center, tangents[1], next.isLeftTurn)),
                               points.get(i + 1));

            } else {
                // Last internal waypoint: arc to the final tangent point
                Translation2d lastTangent = TangentCalculator.pointToCircleTangent(
                        points.get(points.size() - 1).pose.getTranslation(),
                        curr.center,
                        radius,
                        curr.isLeftTurn);
                addArcLeg(legs.get(legs.size() - 1).end,
                          toPose(lastTangent, angleToTarget(curr.center, lastTangent, curr.isLeftTurn)),
                          curr, points.get(i));
            }
        }

        // Step B.3: final straight leg to the last waypoint
        addStraightLeg(legs.get(legs.size() - 1).end,
                       points.get(points.size() - 1).pose,
                       points.get(points.size() - 1));
    }

    // =========================================================================
    //  PATH TRACKING
    // =========================================================================

    /**
     * Called every execute() cycle. Returns the ChassisSpeeds needed to
     * follow the current leg. Automatically advances to the next leg when done.
     *
     * @param currentSpeeds robot-relative speeds from the drivetrain
     * @param pose          current robot pose from odometry
     */
    public ChassisSpeeds calculateSpeeds(ChassisSpeeds currentSpeeds, Pose2d pose) {
        if (finished || legs.isEmpty()) return new ChassisSpeeds();

        Leg leg = legs.get(currentLegIndex);
        CalculateResult result;

        if (leg.type == Leg.LegType.STRAIGHT) {
            result = trackStraight(leg, currentSpeeds, pose);
        } else {
            result = trackArc(leg, currentSpeeds, pose);
        }

        // Advance to next leg when current leg is complete
        if (result.distanceLeft <= 0) {
            currentLegIndex++;
            if (currentLegIndex >= legs.size()) {
                finished = true;
                return new ChassisSpeeds();
            }
        }

        return toChassisSpeeds(result, currentSpeeds, pose);
    }

    public boolean isFinished() { return finished; }

    // =========================================================================
    //  STRAIGHT LEG TRACKING
    // =========================================================================

    /**
     * Tracks a straight leg.
     * - Computes distance and angle to the end point.
     * - Ends the leg if close enough or angle error is too large.
     * - Uses a "double correction" heading to guide the robot back to the base line.
     * - Computes velocity using the trapezoidal profile.
     */
    private CalculateResult trackStraight(Leg leg, ChassisSpeeds currentSpeeds, Pose2d pose) {
        Translation2d vec = leg.end.getTranslation().minus(pose.getTranslation());

        double distanceLeft  = vec.getNorm();
        double angleToTarget = vec.getAngle().getRadians();
        double baseAngle     = leg.start.getRotation().getRadians();
        double angleError    = baseAngle - angleToTarget;
        double endAngle      = leg.end.getRotation().getRadians();

        // End condition: arrived or angle error too large
        if (distanceLeft < ARRIVE_THRESHOLD || Math.abs(angleError) > ANGLE_ERR_MAX) {
            return new CalculateResult(leg.endVelocity, endAngle, 0);
        }

        double currentV = Math.hypot(currentSpeeds.vxMetersPerSecond,
                                     currentSpeeds.vyMetersPerSecond);
        double v = Trapezoid.calculate(currentV, leg.endVelocity,
                                       leg.maxVelocity, leg.maxAcceleration, distanceLeft);

        // Double-correction heading: steers back toward the base line
        double angle = 2 * angleToTarget - endAngle;

        return new CalculateResult(v, angle, distanceLeft);
    }

    // =========================================================================
    //  ARC TRACKING
    // =========================================================================

    /**
     * Tracks an arc leg.
     * - Computes the base heading (tangent direction at current position).
     * - Ends the leg when the base heading is close to the target heading.
     * - Corrects the effective radius based on how far the robot is from the ideal circle.
     * - Computes angular rate (omega) = velocity / corrected_radius.
     */
    private CalculateResult trackArc(Leg leg, ChassisSpeeds currentSpeeds, Pose2d pose) {
        Translation2d centerToPos = leg.arcCenter.minus(pose.getTranslation());

        // Tangent direction at the robot's current position on the circle
        double baseHeading   = centerToPos.getAngle().getRadians()
                               + (leg.isLeftTurn ? Math.PI / 2 : -Math.PI / 2);
        double targetHeading = leg.end.getRotation().getRadians();

        // End condition: heading is close enough to the target
        if (Math.abs(baseHeading - targetHeading) < ARC_DONE_THRESHOLD) {
            return new CalculateResult(leg.endVelocity, targetHeading, 0);
        }

        // Corrected radius: pulls the robot back toward the ideal circle
        double r = centerToPos.getNorm();
        r = leg.arcRadius * 2 - r;

        double distanceLeft = Math.abs(targetHeading - baseHeading) * leg.arcRadius;

        double currentV = Math.hypot(currentSpeeds.vxMetersPerSecond,
                                     currentSpeeds.vyMetersPerSecond);
        double v = Trapezoid.calculate(currentV, leg.endVelocity,
                                       leg.maxVelocity, leg.maxAcceleration, distanceLeft);

        // omega = v / r, then advance the heading by omega * dt
        double omega       = v / r;
        double angleChange = omega * CYCLE_TIME;
        double angle       = baseHeading + (leg.isLeftTurn ? angleChange : -angleChange);

        return new CalculateResult(v, angle, distanceLeft);
    }

    // =========================================================================
    //  HELPERS
    // =========================================================================

    /** Converts a CalculateResult into field-relative ChassisSpeeds with heading correction. */
    private ChassisSpeeds toChassisSpeeds(CalculateResult result,
                                           ChassisSpeeds currentSpeeds,
                                           Pose2d pose) {
        double vx = result.velocity * Math.cos(result.heading);
        double vy = result.velocity * Math.sin(result.heading);

        // Simple P controller for robot rotation toward the leg's end heading
        double robotHeading  = pose.getRotation().getRadians();
        double targetHeading = currentLegIndex < legs.size()
                ? legs.get(currentLegIndex).end.getRotation().getRadians()
                : robotHeading;
        double headingError = angleModulus(targetHeading - robotHeading);
        double omega = KP_HEADING * headingError;

        return new ChassisSpeeds(vx, vy, omega);
    }

    private void addStraightLeg(Pose2d start, Pose2d end, TrajectoryPoint tp) {
        legs.add(new Leg(start, end, tp.maxVelocity, tp.maxAcceleration, tp.velocity));
    }

    private void addArcLeg(Pose2d start, Pose2d end, Circle circle, TrajectoryPoint tp) {
        legs.add(new Leg(start, end, circle.center, circle.radius, circle.isLeftTurn,
                         tp.maxVelocity, tp.maxAcceleration, tp.velocity));
    }

    private Pose2d toPose(Translation2d t, Rotation2d r) {
        return new Pose2d(t, r);
    }

    /** Returns the tangent heading at a point on a circle (perpendicular to the radius). */
    private Rotation2d angleToTarget(Translation2d center, Translation2d point, boolean isLeftTurn) {
        Translation2d vec = point.minus(center);
        double angle = vec.getAngle().getRadians() + (isLeftTurn ? Math.PI / 2 : -Math.PI / 2);
        return new Rotation2d(angle);
    }

    /** Wraps angle to the range [-π, π] */
    private static double angleModulus(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /** Returns the built leg list (useful for debugging / visualization). */
    public List<Leg> getLegs() { return legs; }
}
