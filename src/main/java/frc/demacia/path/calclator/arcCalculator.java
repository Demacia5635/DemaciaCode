package frc.demacia.path.calclator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.utils.log.Log;

public class arcCalculator {
    
    public static ArcSegment calclateCenter(Translation2d from, Translation2d midPoint,Translation2d to, double radius){
        double MidPointToFromAngle = from.minus(midPoint).getAngle().getRadians();
        double MidToPointToAngle = to.minus(midPoint).getAngle().getRadians();
        double vecAngle = (MidPointToFromAngle + MidToPointToAngle) / 2;
        // double toMidPointAngle = MidPointToFromAngle + Math.PI;
        double angleDiff = MathUtil.angleModulus(MidToPointToAngle - MidPointToFromAngle);
        boolean isLeftTurn = angleDiff < 0;
        Translation2d center = midPoint.plus(new Translation2d(radius * Math.cos(vecAngle), radius * Math.sin(vecAngle)));
        Log.log("angleDiff: " + angleDiff + " from: " + from + " midPoint: "  + midPoint + " to: " + to + " vecAngle: " + vecAngle + " center: " + center + " isLeftTurn: " + isLeftTurn + " MidPointToFromAngle: " + MidPointToFromAngle + " MidToPointToAngle: " + MidToPointToAngle);
        return new ArcSegment(from, to, center, isLeftTurn, radius);
    }
}
