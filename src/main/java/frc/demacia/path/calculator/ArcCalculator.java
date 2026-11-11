package frc.demacia.path.calculator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.utils.log.Log;

public class ArcCalculator {
    
    public static ArcSegment calculateCenter(Translation2d from, Translation2d midPoint,Pose2d to, double radius){
        double MidPointToFromAngle = from.minus(midPoint).getAngle().getRadians();
        double MidToPointToAngle = to.getTranslation().minus(midPoint).getAngle().getRadians();
        double vecAngle = (MidPointToFromAngle + MidToPointToAngle) / 2;
        if (Math.abs(MidPointToFromAngle - MidToPointToAngle) > Math.PI) vecAngle += Math.PI;
        // double toMidPointAngle = MidPointToFromAngle + Math.PI;
        double angleDiff = MathUtil.angleModulus(MidToPointToAngle - MidPointToFromAngle);
        boolean isLeftTurn = angleDiff < 0;
        Translation2d center = midPoint.plus(new Translation2d(radius * Math.cos(vecAngle), radius * Math.sin(vecAngle)));
        Log.log("angleDiff: " + angleDiff + " from: " + from + " midPoint: "  + midPoint + " to: " + to + " vecAngle: " + vecAngle + " center: " + center + " isLeftTurn: " + isLeftTurn + " MidPointToFromAngle: " + MidPointToFromAngle + " MidToPointToAngle: " + MidToPointToAngle);
        return new ArcSegment(from, to.getTranslation(), center, isLeftTurn, radius, to.getRotation());
    }
}
