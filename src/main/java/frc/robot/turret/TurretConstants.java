package frc.robot.turret;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.robot.turret.subsystems.Turret;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class TurretConstants {
    public static final String TURRET_NAME = "turret";

    public static final class TurretMotorConstants {
        public static final String TURRET_MOTOR_NAME = "turret motor";
        public static final int TURRET_MOTOR_ID = 50;
        public static final Canbus TURRET_MOTOR_CANBUS = Canbus.Rio; // TODO
        public static final boolean TURRET_MOTOR_BRAKE = true;
        public static final boolean TURRET_MOTOR_INVERT = false;
        public static final double TURRET_MOTOR_GEAR_RATIO = 199.111;
        public static final double TURRET_MOTOR_KP = 0.0; // TODO
        public static final double TURRET_MOTOR_KI = 0.0; // TODO
        public static final double TURRET_MOTOR_KD = 0.0; // TODO
        public static final double TURRET_MOTOR_KS = 0.0; // TODO
        public static final double TURRET_MOTOR_KV = 0.0; // TODO
        public static final double TURRET_MOTOR_KA = 0.0; // TODO
        public static final double TURRET_MOTOR_KG = 0.0; // TODO
        public static final double TURRET_MOTOR_KV2 = 0.0; // TODO
        public static final double TURRET_MOTOR_KCOS = 0.0; // TODO
        public static final double TURRET_MOTOR_MAX_VELOCITY = 0.0; // TODO
        public static final double TURRET_MOTOR_MAX_ACCELERATION = 0.0; // TODO
        public static final double TURRET_MOTOR_MAX_JERK = 0.0; // TODO

        public static final TalonFXConfig TURRET_MOTOR_CONFIG = new TalonFXConfig(TURRET_MOTOR_NAME, TURRET_MOTOR_ID, TURRET_MOTOR_CANBUS)
            .withBrake(TURRET_MOTOR_BRAKE)
            .withInvert(TURRET_MOTOR_INVERT)
            .withRadiansMotor(TURRET_MOTOR_GEAR_RATIO)
            .withPID(TURRET_MOTOR_KP, TURRET_MOTOR_KI, TURRET_MOTOR_KD, TURRET_MOTOR_KS, TURRET_MOTOR_KV, TURRET_MOTOR_KA, TURRET_MOTOR_KG, TURRET_MOTOR_KCOS, TURRET_MOTOR_KV2)
            .withMotionParam(TURRET_MOTOR_MAX_VELOCITY, TURRET_MOTOR_MAX_ACCELERATION, TURRET_MOTOR_MAX_JERK);

        public static final double TURRET_MOTOR_MIN_LIMIT = 0.0; // TODO
        public static final double TURRET_MOTOR_MAX_LIMIT = 0.0; // TODO
        public static final double TURRET_MOTOR_CALIBRATION_POWER = -0.1; // TODO
        public static final double TURRET_MOTOR_CMD_CALIBRATION_RESET_POS = 0.0; // TODO
        public static final double TURRET_MOTOR_AUTO_CALIBRATION_RESET_POS = 0.0; // TODO
    }

    public static final class TurretMinLimitSwitchConstants {
        public static final String TURRET_MIN_LIMIT_SWITCH_NAME = "turret min Limit Switch";
        public static final int TURRET_MIN_LIMIT_SWITCH_ID = 8;
        public static final boolean TURRET_MIN_LIMIT_SWITCH_INVERT = true;
        public static final LimitSwitchConfig TURRET_MIN_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(TURRET_MIN_LIMIT_SWITCH_NAME,TURRET_MIN_LIMIT_SWITCH_ID)
            .withInvert(TURRET_MIN_LIMIT_SWITCH_INVERT);
    }

    public static enum TurretStates implements MechanismState {
        SHOOTING,
        DELIVERY;

        @Override public double[] getValues() {
            return Turret.getInstance().getTurretValues();
        }
    }
}
