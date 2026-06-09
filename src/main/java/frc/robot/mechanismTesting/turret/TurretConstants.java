package frc.robot.mechanismTesting.turret;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class TurretConstants {

    public static final String NAME = "Turret";
    
    public static final String MOTOR_NAME = "TurretMotor";
    private static final int ID = 50;
    private static final Canbus MOTOR_CANBUS = Canbus.Rio;
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KS = 0;
    private static final double KV = 0;
    private static final double KA = 0;
    private static final double KG = 0;
    public static final TalonFXConfig MOTOR_CONFIG = new TalonFXConfig(ID, MOTOR_CANBUS, MOTOR_NAME)
    .withPID(KP, KI, KD, KS, KV, KA, KG)
    .withInvert(false)
    ;

    public static final double MIN_POSITION = 0;
    public static final String LIMIT_SWITCH_NAME_MIN = "TurretLimitSwitchMin";
    
    private static final int LIMIT_SWITCH_CHANNEL_MIN = 0;
    public static final LimitSwitchConfig LIMIT_SWITCH_CONFIG_MIN = new LimitSwitchConfig(LIMIT_SWITCH_CHANNEL_MIN, LIMIT_SWITCH_NAME_MIN);
    public static final double MAX_POSITION = 0;
    public static final String LIMIT_SWITCH_NAME_MAX = "TurretLimitSwitchMax";
    private static final int LIMIT_SWITCH_CHANNEL_MAX = 0;
    public static final LimitSwitchConfig LIMIT_SWITCH_CONFIG_MAX = new LimitSwitchConfig(LIMIT_SWITCH_CHANNEL_MAX, LIMIT_SWITCH_NAME_MAX);
    
    public static enum TURRET_STATES implements MechanismState{
        Hub, 
        Delivery;

        public double[] getValues(){
            return new double[] {0};
        }
    }
}
