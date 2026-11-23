package frc.demacia.path;

/**
 * Numbers shared by every path. They are not changed per path (see PathConfig for that).
 */
public class PathConstants {
    public static final double DT = 0.02; // s
    public static final double MIN_TURN_FOR_ARC = Math.toRadians(1); // rad
    public static final double MAX_ARC_SWEEP = Math.toRadians(270); // rad
    public static final double MAX_RADIUS_TO_LEG_RATIO = 0.25; // arc radius <= this * shorter leg, so close corners don't overlap
    public static final double MIN_WAYPOINT_DISTANCE = 0.01; // m, closer consecutive waypoints are rejected
    public static final double MAX_VELOCITY_TO_CONSIDER_STOPPED = 0.1; // m/s
}
