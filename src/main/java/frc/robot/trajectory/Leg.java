package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
 
public class Leg {
 
    public enum Type { STRAIGHT, ARC }
 
    public final Type type;
    public final Pose2d start;
    public final Pose2d end;
    public final double maxVelocity;
    public final double maxAcceleration;
    public final double endVelocity;
 
    // arc only
    public final Translation2d arcCenter;
    public final double arcRadius;
    public final boolean isLeftTurn;
 
    public Leg(Pose2d start, Pose2d end,
               double maxVelocity, double maxAcceleration, double endVelocity) {
        this.type            = Type.STRAIGHT;
        this.start           = start;
        this.end             = end;
        this.maxVelocity     = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.endVelocity     = endVelocity;
        this.arcCenter       = null;
        this.arcRadius       = 0;
        this.isLeftTurn      = false;
    }
 
    public Leg(Pose2d start, Pose2d end,
               Translation2d arcCenter, double arcRadius, boolean isLeftTurn,
               double maxVelocity, double maxAcceleration, double endVelocity) {
        this.type            = Type.ARC;
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