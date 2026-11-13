package frc.demacia.path.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.demacia.path.constants.PathConstants;

public final class PathUtils {

    private PathUtils() {}

    /** True if the robot's velocity heading is close enough to the wanted heading to finish an arc. */
    public static boolean isHeadingCloseToTarget(Rotation2d currentVelocityHeading, Rotation2d targetHeading) {
        return Math.abs(currentVelocityHeading.minus(targetHeading).getRadians())
                < PathConstants.MAX_HEADING_ERROR_TO_FINISH_ARC_RAD;
    }
}
