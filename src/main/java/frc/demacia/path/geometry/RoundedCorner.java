package frc.demacia.path.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The result of rounding one corner of the path.
 *
 * @param entry      where the robot leaves the straight line and starts the arc
 * @param exit       where the robot leaves the arc and starts the next straight line
 * @param center     center of the rounding circle
 * @param isLeftTurn turn direction around the circle
 */
public record RoundedCorner(Pose2d entry, Pose2d exit, Translation2d center, boolean isLeftTurn) {}
