package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a single segment of the path — either a straight line or an arc.
 */
public class Leg {

    public enum LegType { STRAIGHT, ARC }

    // --- shared fields ---
    public final LegType type;
    public final Pose2d start;
    public final Pose2d end;
    public final double maxVelocity;      // max allowed velocity on this leg (m/s)
    public final double maxAcceleration;  // max allowed acceleration on this leg (m/s²)
    public final double endVelocity;      // desired velocity at the end of this leg (m/s)

    // --- arc-only fields (null / 0 for straight legs) ---
    public final Translation2d arcCenter;
    public final double arcRadius;
    public final boolean isLeftTurn;

    /** Constructor for a straight leg */
    public Leg(Pose2d start, Pose2d end,
               double maxVelocity, double maxAcceleration, double endVelocity) {
        this.type            = LegType.STRAIGHT;
        this.start           = start;
        this.end             = end;
        this.maxVelocity     = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.endVelocity     = endVelocity;
        this.arcCenter       = null;
        this.arcRadius       = 0;
        this.isLeftTurn      = false;
    }

    /** Constructor for an arc leg */
    public Leg(Pose2d start, Pose2d end,
               Translation2d arcCenter, double arcRadius, boolean isLeftTurn,
               double maxVelocity, double maxAcceleration, double endVelocity) {
        this.type            = LegType.ARC;
        this.start           = start;
        this.end             = end;
        this.maxVelocity     = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.endVelocity     = endVelocity;
        this.arcCenter       = arcCenter;
        this.arcRadius       = arcRadius;
        this.isLeftTurn      = isLeftTurn;
    }

    @Override
    public String toString() {
        return String.format("[%s] %s -> %s", type, start.getTranslation(), end.getTranslation());
    }
}
