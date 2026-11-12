package frc.robot.Gripper.subsystems.constants;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.DigitalEncoderConfig;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.LimitSwitchConfig;
import frc.demacia.utils.sensors.OpticalSensorConfig;
import frc.demacia.utils.sensors.UltraSonicSensorConfig;

public class ConstantsGripper {

    public static final int LIMIT_SWITCH_ID = -1;
    public static final boolean IS_BRAKE_LIMIT_SWITCH = true;
    public static final String LIMIT_SWITCH_NAME = "LIMIT_SWITCH_NAME";
    public static final LimitSwitchConfig LIMIT_SWITCH_CONFIG_ROLL = new LimitSwitchConfig(LIMIT_SWITCH_ID,
            LIMIT_SWITCH_NAME);

    public static final int LIMIT_SWITCH_ID2 = -1;
    public static final String LIMIT_SWITCH_NAME2 = "LIMIT_SWITCH_NAME2";
    public static final LimitSwitchConfig LIMIT_SWITCH_CONFIG_PITCH = new LimitSwitchConfig(LIMIT_SWITCH_ID2,
            LIMIT_SWITCH_NAME2);
    // Private PID & Feedforward Const ants
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KS = 0;
    private static final double KV = 0;
    private static final double KA = 0;
    private static final double KG = 0;

    // Private Motion Magic & Motor Limits
    private static final double MOTION_MAGIC_VELOCITY = 0;
    private static final double MOTION_MAGIC_ACCELERATION = 0;
    private static final double MOTION_MAGIC_JERK = 0;

    private static final boolean IS_BRAKE = true;
    private static final boolean IS_INVERTED = false;
    private static final double GEAR_RATIO = 1;
    private static final double RAMP_TIME = 0;
    private static final double MAX_VOLT = 12;
    private static final double MAX_CURRENT = 40;

    // Main Motor Configuration
    public static final int kKrakenID = -1;
    public static final boolean kIsBrake = true;
    public static final Canbus kSKrakencanbus = Canbus.Rio;
    public static final String kKrakename = "Kraken_Motor_1";

    public static final TalonFXConfig CONFIG = new TalonFXConfig(kKrakenID, kSKrakencanbus, kKrakename)
            .withPID(KP, KI, KD, KS, KV, KA, KG)
            .withMotionParam(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
            .withBrake(IS_BRAKE)
            .withInvert(IS_INVERTED)
            .withRadiansMotor(GEAR_RATIO)
            .withRampTime(RAMP_TIME)
            .withVolts(MAX_VOLT)
            .withCurrent(MAX_CURRENT);

    // Motor 2 Configuration
    public static final int kKrakenID2 = -1;
    public static final boolean kIsBrake2 = true;
    public static final Canbus kSKrakencanbus2 = Canbus.Rio;
    public static final String kKrakename2 = "Kraken_Motor_2";

    public static final int ID = kKrakenID2;
    public static final Canbus CAN_BUS = kSKrakencanbus2;
    public static final String NAME = kKrakename2;

    public static final TalonFXConfig CONFIG2 = new TalonFXConfig(ID, CAN_BUS, NAME)
            .withPID(KP, KI, KD, KS, KV, KA, KG)
            .withMotionParam(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK)
            .withBrake(IS_BRAKE)
            .withInvert(IS_INVERTED)
            .withRadiansMotor(GEAR_RATIO)
            .withRampTime(RAMP_TIME)
            .withVolts(MAX_VOLT)
            .withCurrent(MAX_CURRENT);

    // Public PID Constants
    public static final double Kp = -1;
    public static final double Ki = -1;
    public static final double Kd = -1;
    public static final double Ks = -1;
    public static final double Kv = -1;
    public static final double Ka = -1;
    public static final double Kg = -1;

    

    public static final int kSnowblowerID = 32;
    public static final String kSnowBlowerName = "Snow_Blower";
    public static final double DISTANCE_TO_BOTTLE = 0.07;
    public static final double DISTANCE_TO_BOTTLE_CLAMP = 0.1;

    public static final TalonSRXConfig kSnowBlowerConfig = new TalonSRXConfig(
        kSnowblowerID,
        kSnowBlowerName
    );
    public static final int K_OPTIC_SENSOR_ID = 8;
    public static final String kOpticSensorNAME = "Optic_Sensor"; 
    public static final OpticalSensorConfig kOpticSensorConfig = new OpticalSensorConfig(kOpticSensorNAME, K_OPTIC_SENSOR_ID);
      public static final UltraSonicSensorConfig kUltraSonicSensorConfig = new UltraSonicSensorConfig(8, 9, "UltraSonic_Sensor");
}
