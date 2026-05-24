package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
 
public class Leg {

 
    private final Pose2d start;
    private final Pose2d end;
    private final double maxVelocity;
    private final double maxAcceleration;
    private final double endVelocity;
 
    private final Circle circleStart;
    private final Circle circleEnd;

    private final Translation2d arcCenter;
    private final double arcRadius;
    private final boolean isLeftTurn;
 
    public Leg(Pose2d start, Pose2d end, double maxVelocity, double maxAcceleration, double endVelocity) {
        this.start           = start;
        this.end             = end;
        this.maxVelocity     = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.endVelocity     = endVelocity;
        this.arcCenter       = null;
        this.arcRadius       = 0;
        this.isLeftTurn      = false;
    }
 
    public Leg(Pose2d start, Pose2d end,Translation2d arcCenter, double arcRadius, boolean isLeftTurn,double maxVelocity, double maxAcceleration, double endVelocity) {
        this.start           = start;
        this.end             = end;
        this.maxVelocity     = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.endVelocity     = endVelocity;
        this.arcCenter       = arcCenter;
        this.arcRadius       = arcRadius;
        this.isLeftTurn      = isLeftTurn;
    }
 
        public double getDistance() {
            if (isLeftTurn){
                return arcCenter
            }
        }
}