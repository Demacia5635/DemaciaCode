package frc.demacia.path.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.demacia.path.constans.PathConstants;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.SegmantBase;

public class PathUtils {
    public static boolean isLineSegment(SegmantBase segmant) {
        // Check if the start and end poses are the same
        return segmant instanceof LineSegment;
        
    }

    public static boolean isVelocityHeadingInRange(Rotation2d currentVelocityHeading,
            Rotation2d wantedVelocityHeading) {
        return Math.abs(currentVelocityHeading.minus(wantedVelocityHeading)
                .getRadians()) < PathConstants.MAX_VELOCITY_HEADING_TO_FINISH_SEGMENT;
    }
}
