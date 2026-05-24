package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
 
import java.util.ArrayList;
import java.util.List;
 
public class DemaciaTrajectory {
 
    private final List<Leg> legs = new ArrayList<>();
 
    public List<Leg> build(List<Pose2d> points, double[] velocities,
                           double[] maxVelocities, double[] maxAccelerations,
                           double radius) {
        legs.clear();
        if (points.size() < 2) return legs;
 
        // compute circle centers for every internal waypoint
        Circle[] circles = new Circle[points.size()];
        for (int i = 1; i < points.size() - 1; i++) {
            circles[i] = Circle.calculate(
                    points.get(i - 1).getTranslation(),
                    points.get(i + 1).getTranslation(),
                    points.get(i).getTranslation(),
                    radius);
        }
 
        // straight leg from start to first tangent point
        Translation2d firstTangent = pointToCircleTangent(
                points.get(0).getTranslation(), circles[1], radius);
        addStraight(points.get(0),
                    new Pose2d(firstTangent, points.get(0).getRotation()),
                    maxVelocities[1], maxAccelerations[1], velocities[1]);
 
        for (int i = 1; i < points.size() - 1; i++) {
            Circle curr = circles[i];
            boolean hasNext = i + 1 < points.size() - 1;
 
            if (hasNext) {
                Circle next = circles[i + 1];
                Translation2d[] tangents;
 
                if (curr.isLeftTurn == next.isLeftTurn) {
                    tangents = sameTurnTangents(curr, next, radius);
                } else {
                    tangents = oppositeTurnTangents(curr, next, radius);
                }
 
                addArc(legs.get(legs.size() - 1).end,
                       new Pose2d(tangents[0], tangentHeading(curr, tangents[0])),
                       curr, maxVelocities[i], maxAccelerations[i], velocities[i]);
 
                addStraight(legs.get(legs.size() - 1).end,
                            new Pose2d(tangents[1], tangentHeading(next, tangents[1])),
                            maxVelocities[i + 1], maxAccelerations[i + 1], velocities[i + 1]);
            } else {
                Translation2d lastTangent = pointToCircleTangent(
                        points.get(points.size() - 1).getTranslation(), curr, radius);
                addArc(legs.get(legs.size() - 1).end,
                       new Pose2d(lastTangent, tangentHeading(curr, lastTangent)),
                       curr, maxVelocities[i], maxAccelerations[i], velocities[i]);
            }
        }
 
        // final straight to last waypoint
        addStraight(legs.get(legs.size() - 1).end,
                    points.get(points.size() - 1),
                    maxVelocities[points.size() - 1],
                    maxAccelerations[points.size() - 1],
                    velocities[points.size() - 1]);
 
        return legs;
    }
 
    // --- tangent helpers ---
 
    private Translation2d pointToCircleTangent(Translation2d p, Circle c, double radius) {
        Translation2d vec = p.minus(c.center);
        double d = vec.getNorm();
        double base = vec.getAngle().getRadians();
        double alpha = Math.acos(radius / d);
        double angle = base + (c.isLeftTurn ? -alpha : alpha);
        return c.center.plus(new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle)));
    }
 
    private Translation2d[] sameTurnTangents(Circle c1, Circle c2, double radius) {
        Translation2d vec = c2.center.minus(c1.center);
        double angle = vec.getAngle().getRadians() + (c1.isLeftTurn ? -Math.PI / 2 : Math.PI / 2);
        Translation2d offset = new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle));
        return new Translation2d[]{ c1.center.plus(offset), c2.center.plus(offset) };
    }
 
    private Translation2d[] oppositeTurnTangents(Circle c1, Circle c2, double radius) {
        Translation2d vec = c2.center.minus(c1.center);
        double d = vec.getNorm();
        double base = vec.getAngle().getRadians();
        double a = Math.acos(radius * 2 / d);
        double angle = base + (c1.isLeftTurn ? +a : -a);
        Translation2d offset = new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle));
        return new Translation2d[]{ c1.center.plus(offset), c2.center.minus(offset) };
    }
 
    private Rotation2d tangentHeading(Circle c, Translation2d point) {
        Translation2d vec = point.minus(c.center);
        double angle = vec.getAngle().getRadians() + (c.isLeftTurn ? Math.PI / 2 : -Math.PI / 2);
        return new Rotation2d(angle);
    }
 
    // --- leg builders ---
 
    private void addStraight(Pose2d start, Pose2d end,
                              double maxV, double maxA, double endV) {
        legs.add(new Leg(start, end, maxV, maxA, endV));
    }
 
    private void addArc(Pose2d start, Pose2d end, Circle c,
                        double maxV, double maxA, double endV) {
        legs.add(new Leg(start, end, c.center, c.radius, c.isLeftTurn, maxV, maxA, endV));
    }
}