package frc.demacia.path.builder;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.path.geometry.CornerRounder;
import frc.demacia.path.geometry.RoundedCorner;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.PathSegment;
import frc.demacia.utils.log.Log;

/**
 * Turns a list of waypoints into the segments the robot follows:
 * line, arc, line, arc, ..., line. Every inner waypoint becomes a rounded corner.
 */
public final class TrajectoryBuilder {

    private TrajectoryBuilder() {}

    /** @return the segments, or an empty list if there are fewer than 2 waypoints */
    public static List<PathSegment> build(List<Pose2d> waypoints, double cornerRadiusMeters) {
        List<PathSegment> segments = new ArrayList<>();

        if (waypoints.size() < 2) {
            Log.log("Not enough points to build a trajectory");
            return segments;
        }

        Translation2d lineStart = waypoints.get(0).getTranslation();

        for (int i = 1; i < waypoints.size() - 1; i++) {
            RoundedCorner corner = CornerRounder.round(
                    waypoints.get(i - 1).getTranslation(),
                    waypoints.get(i),
                    waypoints.get(i + 1).getTranslation(),
                    cornerRadiusMeters);

            segments.add(new LineSegment(lineStart, corner.entry()));
            segments.add(new ArcSegment(corner.entry().getTranslation(), corner.exit(),
                    corner.center(), corner.isLeftTurn(), cornerRadiusMeters));

            lineStart = corner.exit().getTranslation();
        }

        segments.add(new LineSegment(lineStart, waypoints.get(waypoints.size() - 1)));

        for (int i = 0; i < segments.size(); i++) {
            Log.log("pathSegments.get(" + i + "): " + segments.get(i));
        }
        return segments;
    }
}
