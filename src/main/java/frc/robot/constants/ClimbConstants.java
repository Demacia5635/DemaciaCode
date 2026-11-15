package frc.robot.constants;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class ClimbConstants {
    public static final Canbus CANBUS = Canbus.Rio;
    public static final double GEAR_RATIO = 0.0;
    public static final double Kp = 0.0;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;
    public static final double Ks = 0.0;
    public static final double Kv = 0.0;
    public static final double Ka = 0.0;
    public static final double Kg = 0.0;
    public static final double Kcos = 0.0;
    public static final double Kv2 = 0.0;

    public static final int LEFT_LIMIT_SWITCH_CHANNEL = 0;
    public static final String LEFT_LIMIT_SWITCH_NAME = "Left Limit Switch";

    public static final int RIGHT_LIMIT_SWITCH_CHANNEL = 1;
    public static final String RIGHT_LIMIT_SWITCH_NAME = "Right Limit Switch";

    public static final int LEFT_MOTOR_ID = 0;
    public static final String LEFT_MOTOR_NAME = "Left Motor";
    
    public static final int RIGHT_MOTOR_ID = 0;
    public static final String RIGHT_MOTOR_NAME = "Right Motor";

    public static final TalonFXConfig LEFT_MOTOR_CONFIG = new TalonFXConfig(LEFT_MOTOR_NAME, LEFT_MOTOR_ID, CANBUS)
    .withPID(Kp, Ki, Kd, Ks, Kv, Ka, Kg, Kcos, Kv2)
    .withRadiansMotor(GEAR_RATIO);

    public static final TalonFXConfig RIGHT_MOTOR_CONFIG = new TalonFXConfig(RIGHT_MOTOR_NAME, RIGHT_MOTOR_ID, CANBUS)
    .withPID(Kp, Ki, Kd, Ks, Kv, Ka, Kg, Kcos, Kv2)
    .withRadiansMotor(GEAR_RATIO);

    public static final LimitSwitchConfig LEFT_SWITCH_CONFIG = new LimitSwitchConfig(LEFT_LIMIT_SWITCH_NAME, LEFT_LIMIT_SWITCH_CHANNEL);
    public static final LimitSwitchConfig RIGHT_SWITCH_CONFIG = new LimitSwitchConfig(RIGHT_LIMIT_SWITCH_NAME, RIGHT_LIMIT_SWITCH_CHANNEL);

    public static final double OPEN_POSITION = 0.0;
    public static final double BALANCE_START_DISTANCE = 0.0;
    public static final double BALANCE_FACTOR = 0.0;
    public static final double MAX_BALANCE_CORRECTION = 0.0;
    public static final double DRIVE_TIME = 0.0;
    public static final double DRIVE_SPEED = 0.0;
    public static final double POSITION_TOLERANCE = 0.0;
    public static final double BALANCE_TOLERANCE = Math.toRadians(10);

    public enum ClimbState {
        OPENING,
        DRIVING,
        CLIMBING,
        BALANCING,
        FINISHED,
        AUTO_CLIMB
    }

}
