package frc.demacia.path.constants;

/** Tuning constants for path generation and following. */
public final class PathConstants {

    private PathConstants() {}

    /** Duration of one robot control-loop cycle. */
    public static final double CONTROL_LOOP_PERIOD_SECONDS = 0.02;

    // TODO: not used anywhere in the path package - rename or remove once its purpose is clear.
    public static final double TOLERANCE = 0.5;

    public static final double MAX_VELOCITY_MPS = 4;
    public static final double MAX_ACCELERATION_MPS2 = 3;
    public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 2;
    public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 = 0.6;

    /** An arc segment is finished when the velocity heading is within this angle of the arc's exit direction. */
    public static final double MAX_HEADING_ERROR_TO_FINISH_ARC_RAD = Math.toRadians(10);

    /** A line segment is finished when the robot is this close to its end point (last segment of the path). */
    public static final double FINAL_POINT_POSITION_THRESHOLD_METERS = 0.1;

    /** A line segment is finished when the robot is this close to its end point (any segment except the last). */
    public static final double MID_PATH_POSITION_THRESHOLD_METERS = 0.1;

    /** A line segment is also finished when the end point is behind the robot by more than this angle. */
    public static final double MAX_ANGLE_TO_CONSIDER_POINT_PASSED_RAD = Math.toRadians(60);

    // TODO: CHANGE THIS
    public static final double CORNER_ARC_RADIUS_METERS = 0.5;
}
