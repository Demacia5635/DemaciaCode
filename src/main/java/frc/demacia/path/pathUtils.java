package frc.demacia.path;

import edu.wpi.first.math.geometry.Rotation2d;

public class pathUtils {
    public static boolean isLineSegment(SegmantBase segmant) {
        // Check if the start and end poses are the same
        return segmant instanceof LineSegment;
        
    }

    public static boolean isVelocityHeadingInRange(Rotation2d currentVelocityHeading,
            Rotation2d wantedVelocityHeading) {
        return Math.abs(currentVelocityHeading.minus(wantedVelocityHeading)
                .getRadians()) < pathConstans.MAX_VELOCITY_HEADING_TO_FINISH_SEGMENT;
    }
}
