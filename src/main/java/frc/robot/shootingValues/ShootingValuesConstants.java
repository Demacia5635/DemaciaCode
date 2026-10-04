package frc.robot.shootingValues;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.LookUpTable;
import frc.robot.Field;

public class ShootingValuesConstants {
    public static final LookUpTable LOOK_UP_TABLE = new LookUpTable(2);
    static {
        LOOK_UP_TABLE.add(0.93, 9, Math.toRadians(20));
        LOOK_UP_TABLE.add(2.16, 9.5,Math.toRadians(30));
        LOOK_UP_TABLE.add(2.43,9.5,Math.toRadians(40));
    }

    public static final Translation2d TURRET_POSE_ON_ROBOT = new Translation2d(0.24, 0);

    public static final Translation2d HUB_POSE_BLUE = Field.HubBlue.CENTER;
    public static final Translation2d HUB_POSE_RED = Field.HubRed.CENTER;

    public static final double PREDICTING_TIME = 0.04;
    public static final double MOTOR_VEL_TO_BALL_VEL = 0.48;

    public static final double SPIN_CORRECTION_GAIN = 0; //0.0001; //TODO
}
