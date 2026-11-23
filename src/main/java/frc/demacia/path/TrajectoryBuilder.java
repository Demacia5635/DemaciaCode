package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.BaseSegment;
import frc.demacia.utils.log.Log;

/**
 * Turns a list of waypoints into lines and arcs, once, before the robot moves.
 * PathFollower then drives them every 20 ms.
 *
 * <p>Steps: round each corner with a circle, connect the circles with straight lines,
 * remove arcs that do not fit, then plan the speeds backward from the end so the robot
 * can always brake in time.
 *
 * <pre>{@code
 * var points = List.of(new Waypoint(startPose),
 *         new Waypoint(pickupPose).withMaxCornerSpeed(2.0), // corner speed limit, m/s
 *         new Waypoint(scorePose));
 * DemaciaTrajectory path = TrajectoryBuilder.build(points); // or build(points, new PathConfig().withMaxSpeed(3.0))
 * }</pre>
 */
public class TrajectoryBuilder {

    /**
     * Builds a trajectory with the default PathConfig.
     *
     * @param points the route in driving order (field poses; the first point's heading is ignored)
     * @return the trajectory (empty if there are fewer than 2 points)
     * @throws IllegalArgumentException if a corner speed is bad or two waypoints are closer than 1 cm
     */
    public static DemaciaTrajectory build(List<Waypoint> points) {
        return build(points, new PathConfig());
    }

    /**
     * Builds a trajectory. The config values are copied into it now,
     * so changing the config afterwards does not change this trajectory.
     *
     * @param points the route in driving order (field poses; the first point's heading is ignored)
     * @param config speed limits and distances (m, m/s, m/s^2, rad, rad/s)
     * @return the trajectory (empty if there are fewer than 2 points)
     * @throws IllegalArgumentException if a config value is not finite and above 0,
     *         if a corner speed is negative, NaN, or set on the first or last point,
     *         or if two consecutive waypoints are closer than 1 cm
     */
    public static DemaciaTrajectory build(List<Waypoint> points, PathConfig config) {
        checkConfig(config);
        checkWaypoints(points);

        List<BaseSegment> segments = new ArrayList<>();
        if (points.size() < 2) {
            Log.log("Not enough points to build a trajectory");
        } else {
            buildSegments(points, config, segments);
            for (int i = 0; i < segments.size(); i++) {
                Log.log("segments.get(" + i + "): " + segments.get(i));
            }
        }
        return new DemaciaTrajectory(segments, config.maxAngularSpeed, config.maxAngularAccel,
                config.finalHeadingTolerance);
    }

    // fails early at build time, so a bad number cannot cause strange driving later
    private static void checkConfig(PathConfig config) {
        double[] values = { config.maxSpeed, config.maxAccel, config.maxLateralAccel, config.maxAngularSpeed,
                config.maxAngularAccel, config.maxPositionThreshold, config.finalPositionThreshold,
                config.passedPointAngle, config.finalHeadingTolerance };
        for (double value : values) {
            if (!Double.isFinite(value) || value <= 0) {
                throw new IllegalArgumentException("path config values must be finite and > 0, got " + value);
            }
        }
    }

    private static void checkWaypoints(List<Waypoint> points) {
        int size = points.size();
        for (int i = 0; i < size; i++) {
            double maxCornerSpeed = points.get(i).getMaxCornerSpeed();
            boolean isFirstOrLast = i == 0 || i == size - 1;
            // the first and last point have no corner, so they can't limit the speed
            // NaN is not equal to anything, so it needs its own check
            boolean isBadSpeed = Double.isNaN(maxCornerSpeed) || maxCornerSpeed < 0
                    || (isFirstOrLast && maxCornerSpeed != Double.POSITIVE_INFINITY);
            if (isBadSpeed) {
                throw new IllegalArgumentException("bad max corner speed " + maxCornerSpeed + " at waypoint " + i);
            }
        }
        for (int i = 1; i < size; i++) {
            double distance = points.get(i).getTranslation().getDistance(points.get(i - 1).getTranslation());
            if (distance < PathConstants.MIN_WAYPOINT_DISTANCE) {
                throw new IllegalArgumentException("waypoints " + (i - 1) + " and " + i + " are too close");
            }
        }
    }

    // The numbers we know about one segment before it is created
    private static class SegmentSpeeds {
        double endVelocity; // m/s
        double maxVelocity; // m/s
        double length; // m

        SegmentSpeeds(double endVelocity, double maxVelocity, double length) {
            this.endVelocity = endVelocity;
            this.maxVelocity = maxVelocity;
            this.length = length;
        }
    }

    // The path is: line, arc, line, arc, ..., line. A point with no arc has only a line before it.
    // Each array below has one value for every waypoint.
    private static void buildSegments(List<Waypoint> points, PathConfig config, List<BaseSegment> segments) {
        int size = points.size();
        Translation2d[] centers = new Translation2d[size]; // the point itself when it has no arc
        double[] signedRadius = new double[size]; // m, positive = left arc, negative = right arc, 0 = no arc
        double[] velocities = new double[size]; // m/s, when passing the point
        Translation2d[] entry = new Translation2d[size]; // where the leg from the previous point ends
        Translation2d[] exit = new Translation2d[size]; // where the leg to the next point starts

        for (int i = 0; i < size; i++) {
            centers[i] = points.get(i).getTranslation();
        }

        // the corner circle of every point
        for (int i = 1; i < size - 1; i++) {
            double maxCornerSpeed = Math.min(points.get(i).getMaxCornerSpeed(), config.maxSpeed);
            RoundedCorner corner = RoundedCorner.fromWaypoints(points.get(i - 1).getTranslation(), points.get(i),
                    points.get(i + 1).getTranslation(), maxCornerSpeed, config);
            centers[i] = corner.center;
            signedRadius[i] = corner.signedRadius;
            velocities[i] = corner.speed;
        }

        // Every leg is the line that touches both of its corner circles.
        // An arc that can't be built is removed and its point becomes a plain point.
        // Then the legs are built again.
        boolean removedArc = true;
        while (removedArc) {
            removedArc = buildLegsOrRemoveArc(points, centers, signedRadius, entry, exit);
            if (!removedArc) {
                removedArc = removeInvalidArcs(points, centers, signedRadius, entry, exit);
            }
        }

        // Speed plan, in path order: one entry per line and one per arc.
        // Lines may go as fast as maxSpeed. Arcs only go at their corner speed.
        List<SegmentSpeeds> segmentSpeeds = new ArrayList<>(); // in path order
        for (int i = 1; i < size; i++) {
            double lineLength = entry[i].minus(exit[i - 1]).getNorm();
            segmentSpeeds.add(new SegmentSpeeds(velocities[i], config.maxSpeed, lineLength));
            if (signedRadius[i] != 0) {
                double radius = Math.abs(signedRadius[i]);
                double arcLength = radius * PathUtils.arcSweep(entry[i], exit[i], centers[i], signedRadius[i] > 0);
                segmentSpeeds.add(new SegmentSpeeds(velocities[i], velocities[i], arcLength));
            }
        }
        limitEndVelocities(segmentSpeeds, config.maxAccel);

        // Only now are the speeds final. We create the segments last, because
        // a segment cannot change its speeds after it is created.
        int speedIndex = 0;
        for (int i = 1; i < size; i++) {
            Rotation2d heading = points.get(i).getRotation();

            double lineEndVelocity = segmentSpeeds.get(speedIndex).endVelocity;
            speedIndex++;
            segments.add(new LineSegment(exit[i - 1], new Pose2d(entry[i], heading), lineEndVelocity, config.maxSpeed, config));

            if (signedRadius[i] != 0) {
                double arcEndVelocity = segmentSpeeds.get(speedIndex).endVelocity;
                speedIndex++;
                segments.add(new ArcSegment(entry[i], new Pose2d(exit[i], heading), centers[i], signedRadius[i] > 0,
                        Math.abs(signedRadius[i]), arcEndVelocity, velocities[i], config));
            }
        }
    }

    // Builds the legs from the first one on and fills entry and exit.
    // Stops at the first leg that can't be built, removes the arcs at both of its points and returns true.
    // Returns false when every leg was built.
    private static boolean buildLegsOrRemoveArc(List<Waypoint> points, Translation2d[] centers, double[] signedRadius,
            Translation2d[] entry, Translation2d[] exit) {
        for (int i = 0; i < points.size() - 1; i++) {
            List<Translation2d> leg = PathUtils.tangentPoints(centers[i], signedRadius[i], centers[i + 1], signedRadius[i + 1]);
            if (leg.isEmpty()) {
                makePlainPoint(i, points, centers, signedRadius);
                makePlainPoint(i + 1, points, centers, signedRadius);
                return true;
            }
            exit[i] = leg.get(0);
            entry[i + 1] = leg.get(1);
        }
        return false;
    }

    // Removes every arc that is not valid. Returns true if it removed at least one.
    private static boolean removeInvalidArcs(List<Waypoint> points, Translation2d[] centers, double[] signedRadius,
            Translation2d[] entry, Translation2d[] exit) {
        boolean removedArc = false;
        for (int i = 1; i < points.size() - 1; i++) {
            boolean hasArc = signedRadius[i] != 0;
            if (hasArc && !isArcValid(centers[i], signedRadius[i] > 0, entry[i], points.get(i).getTranslation(), exit[i])) {
                Log.log("removed the arc of point " + i);
                makePlainPoint(i, points, centers, signedRadius);
                removedArc = true;
            }
        }
        return removedArc;
    }

    // removes the arc of point i
    private static void makePlainPoint(int i, List<Waypoint> points, Translation2d[] centers, double[] signedRadius) {
        centers[i] = points.get(i).getTranslation();
        signedRadius[i] = 0;
    }

    // the arc must go from the entry through its point to the exit without going almost all the way around
    private static boolean isArcValid(Translation2d center, boolean isLeft, Translation2d entry, Translation2d point, Translation2d exit) {
        double entryAngle = entry.minus(center).getAngle().getRadians();
        double pointAngle = point.minus(center).getAngle().getRadians();
        double exitAngle = exit.minus(center).getAngle().getRadians();
        double entryToPoint = MathUtil.inputModulus(isLeft ? pointAngle - entryAngle : entryAngle - pointAngle, 0, 2 * Math.PI);
        double pointToExit = MathUtil.inputModulus(isLeft ? exitAngle - pointAngle : pointAngle - exitAngle, 0, 2 * Math.PI);
        return entryToPoint + pointToExit < PathConstants.MAX_ARC_SWEEP;
    }

    // Backward braking pass: go from the last segment to the first.
    // A segment may only end as fast as the next one can still brake from, using the
    // stopping formula v_end = sqrt(v_next^2 + 2 * a * L). It comes from v^2 = v0^2 + 2*a*L:
    // the speed we can lose over the next segment's length L at braking rate a.
    // It is also capped at the next segment's max speed.
    private static void limitEndVelocities(List<SegmentSpeeds> segmentSpeeds, double maxAccel) {
        for (int i = segmentSpeeds.size() - 2; i >= 0; i--) {
            SegmentSpeeds segment = segmentSpeeds.get(i);
            SegmentSpeeds next = segmentSpeeds.get(i + 1);
            double reachable = Math.sqrt(next.endVelocity * next.endVelocity + 2 * maxAccel * next.length);
            segment.endVelocity = Math.min(segment.endVelocity, Math.min(next.maxVelocity, reachable));
        }
    }
}
