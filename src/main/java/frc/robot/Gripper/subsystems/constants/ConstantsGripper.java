package frc.robot.Gripper.subsystems.constants;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;
import frc.demacia.utils.sensors.UltraSonicSensorConfig;

public class ConstantsGripper {

    //---------------------------------------------------------all----------------------------------------
    public static final boolean Is_Right_Calibraete = false; 
    public static final boolean Is_Left_Calibrate = false;
    int ZeroPointCalinbrate;
    // --------------------------------------------------------- Limit Switches ---------------------------------------------------------
    public static final int KRAKEN_ROLL_LIMIT_SWITCH_ID = -1;
    public static final String KRAKEN_ROLL_LIMIT_SWITCH_NAME = "Kraken_Roll_Limit_Switch";
    public static final boolean KRAKEN_ROLL_LIMIT_SWITCH_INVERTED = false;
    public static final LimitSwitchConfig KRAKEN_ROLL_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(
        KRAKEN_ROLL_LIMIT_SWITCH_ID, KRAKEN_ROLL_LIMIT_SWITCH_NAME
    ).withInvert(KRAKEN_ROLL_LIMIT_SWITCH_INVERTED);

    public static final int KRAKEN_PITCH_LIMIT_SWITCH_ID = -1;
    public static final String KRAKEN_PITCH_LIMIT_SWITCH_NAME = "Kraken_Pitch_Limit_Switch";
    public static final boolean KRAKEN_PITCH_LIMIT_SWITCH_INVERTED = false;
    public static final LimitSwitchConfig KRAKEN_PITCH_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(
        KRAKEN_PITCH_LIMIT_SWITCH_ID, KRAKEN_PITCH_LIMIT_SWITCH_NAME
    ).withInvert(KRAKEN_PITCH_LIMIT_SWITCH_INVERTED);

    // --------------------------------------------------- Shared Kraken Defaults -------------------------------------------------------
    public static final Canbus KRAKEN_CANBUS = Canbus.Rio;

    private static final boolean IS_BRAKE = true;
    private static final boolean IS_INVERTED = false;
    private static final double GEAR_RATIO = 1;
    private static final double RAMP_TIME = 0;
    private static final double MAX_VOLT = 12;
    private static final double MAX_CURRENT = 40;
    public static double  ZeroPositionCalibrte = 0;
    public static final double KRAKEN_PITCH_MIN_RADIANS = -1.57079633 ; // -90 degrees
    public static final double KRAKEN_PITCH_MAX_RADIANS = 1.57079633 ; // 90 degrees

    // --------------------------------------------------- Kraken Roll (TalonFX) --------------------------------------------------------
    public static final double ROLL_KP = 0;
    public static final double ROLL_KI = 0;
    public static final double ROLL_KD = 0;
    public static final double ROLL_KS = 0;
    public static final double ROLL_KV = 0;
    public static final double ROLL_KA = 0;
    public static final double ROLL_KG = 0;

    public static final int KRAKEN_ROLL_ID = -1;
    public static final String KRAKEN_ROLL_NAME = "Kraken_Roll";
    public static final TalonFXConfig KRAKEN_ROLL_CONFIG = new TalonFXConfig(
            KRAKEN_ROLL_ID, KRAKEN_CANBUS, KRAKEN_ROLL_NAME
        )
        .withPID(ROLL_KP, ROLL_KI, ROLL_KD, ROLL_KS, ROLL_KV, ROLL_KA, ROLL_KG)
        .withBrake(IS_BRAKE)
        .withInvert(IS_INVERTED)
        .withRadiansMotor(GEAR_RATIO)
        .withRampTime(RAMP_TIME)
        .withVolts(MAX_VOLT)
        .withCurrent(MAX_CURRENT);

    // -------------------------------------------------- Kraken Pitch (TalonFX) --------------------------------------------------------
    public static final double PITCH_KP = 0;
    public static final double PITCH_KI = 0;
    public static final double PITCH_KD = 0;
    public static final double PITCH_KS = 0;
    public static final double PITCH_KV = 0;
    public static final double PITCH_KA = 0;
    public static final double PITCH_KG = 0;

    public static final int KRAKEN_PITCH_ID = -1;
    public static final String KRAKEN_PITCH_NAME = "Kraken_Pitch";
    public static final TalonFXConfig KRAKEN_PITCH_CONFIG = new TalonFXConfig(
            KRAKEN_PITCH_ID, KRAKEN_CANBUS, KRAKEN_PITCH_NAME
        )
        .withPID(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_KS, PITCH_KV, PITCH_KA, PITCH_KG)
        .withBrake(IS_BRAKE)
        .withInvert(IS_INVERTED)
        .withRadiansMotor(GEAR_RATIO)
        .withRampTime(RAMP_TIME)
        .withVolts(MAX_VOLT)
        .withCurrent(MAX_CURRENT);

    // ------------------------------------------------- Snowblower Claw (TalonSRX) -----------------------------------------------------
    public static final int SNOWBLOWER_CLAW_ID = 32;
    public static final String SNOWBLOWER_CLAW_NAME = "Snowblower_Claw";
    public static final double DISTANCE_TO_BOTTLE = 0.07;
    public static final double DISTANCE_TO_BOTTLE_CLAMP = 0.1;
    public static final TalonSRXConfig SNOWBLOWER_CLAW_CONFIG = new TalonSRXConfig(
        SNOWBLOWER_CLAW_ID, SNOWBLOWER_CLAW_NAME
    );

    // ------------------------------------------------------- Ultrasonic Sensor --------------------------------------------------------

    public static final UltraSonicSensorConfig ULTRASONIC_SENSOR_CONFIG = new UltraSonicSensorConfig(
        8, 9, "UltraSonic_Sensor"
    );
}
