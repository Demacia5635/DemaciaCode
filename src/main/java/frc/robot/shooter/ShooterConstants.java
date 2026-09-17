package frc.robot.shooter;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.robot.shooter.subsystems.Shooter;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class ShooterConstants {
    public static final String SHOOTER_NAME = "shooter";

    public static final class FlywheelConstants {
        public static final String FLYWHEEL_NAME = "flywheel";
        public static final int FLYWHEEL_ID = 60;
        public static final Canbus FLYWHEEL_CANBUS = Canbus.Rio; // TODO
        public static final boolean FLYWHEEL_BRAKE = false;
        public static final boolean FLYWHEEL_INVERT = false;
        public static final double FLYWHEEL_GEAR_RATIO = 1.0;
        public static final double FLYWHEEL_DIAMETER = 0.0508;
        public static final double FLYWHEEL_KP = 0.0; // TODO
        public static final double FLYWHEEL_KI = 0.0; // TODO
        public static final double FLYWHEEL_KD = 0.0; // TODO
        public static final double FLYWHEEL_KS = 0.0; // TODO
        public static final double FLYWHEEL_KV = 0.0; // TODO
        public static final double FLYWHEEL_KA = 0.0; // TODO
        public static final double FLYWHEEL_KG = 0.0; // TODO
        public static final double FLYWHEEL_KV2 = 0.0; // TODO
        public static final double FLYWHEEL_KCOS = 0.0; // TODO

        public static final TalonFXConfig FLYWHEEL_CONFIG = new TalonFXConfig(FLYWHEEL_NAME, FLYWHEEL_ID, FLYWHEEL_CANBUS)
            .withBrake(FLYWHEEL_BRAKE)
            .withInvert(FLYWHEEL_INVERT)
            .withMeterMotor(FLYWHEEL_GEAR_RATIO, FLYWHEEL_DIAMETER)
            .withPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA, FLYWHEEL_KG, FLYWHEEL_KCOS, FLYWHEEL_KV2);

    }

    public static final class HoodConstants {
        public static final String HOOD_NAME = "hood";
        public static final int HOOD_ID = 61;
        public static final Canbus HOOD_CANBUS = Canbus.Rio; // TODO
        public static final boolean HOOD_BRAKE = true;
        public static final boolean HOOD_INVERT = false;
        public static final double HOOD_GEAR_RATIO = 72.0;
        public static final double HOOD_KP = 0.0; // TODO
        public static final double HOOD_KI = 0.0; // TODO
        public static final double HOOD_KD = 0.0; // TODO
        public static final double HOOD_KS = 0.0; // TODO
        public static final double HOOD_KV = 0.0; // TODO
        public static final double HOOD_KA = 0.0; // TODO
        public static final double HOOD_KG = 0.0; // TODO
        public static final double HOOD_KV2 = 0.0; // TODO
        public static final double HOOD_KCOS = 0.0; // TODO
        public static final double HOOD_MAX_VELOCITY = 0.0; // TODO
        public static final double HOOD_MAX_ACCELERATION = 0.0; // TODO
        public static final double HOOD_MAX_JERK = 0.0; // TODO

        public static final TalonFXConfig HOOD_CONFIG = new TalonFXConfig(HOOD_NAME, HOOD_ID, HOOD_CANBUS)
            .withBrake(HOOD_BRAKE)
            .withInvert(HOOD_INVERT)
            .withRadiansMotor(HOOD_GEAR_RATIO)
            .withPID(HOOD_KP, HOOD_KI, HOOD_KD, HOOD_KS, HOOD_KV, HOOD_KA, HOOD_KG, HOOD_KCOS, HOOD_KV2)
            .withMotionParam(HOOD_MAX_VELOCITY, HOOD_MAX_ACCELERATION, HOOD_MAX_JERK);

        public static final double HOOD_MIN_LIMIT = -1000.0; // TODO
        public static final double HOOD_MAX_LIMIT = 1000.0; // TODO
        public static final double HOOD_CALIBRATION_POWER = -0.1; // TODO
        public static final double HOOD_CMD_CALIBRATION_RESET_POS = 0.0; // TODO
        public static final double HOOD_AUTO_CALIBRATION_RESET_POS = 0.0; // TODO
    }

    public static final class FeederConstants {
        public static final String FEEDER_NAME = "feeder";
        public static final int FEEDER_ID = 62;
        public static final Canbus FEEDER_CANBUS = Canbus.Rio; // TODO
        public static final boolean FEEDER_BRAKE = true;
        public static final boolean FEEDER_INVERT = false;

        public static final TalonFXConfig FEEDER_CONFIG = new TalonFXConfig(FEEDER_NAME, FEEDER_ID, FEEDER_CANBUS)
            .withBrake(FEEDER_BRAKE)
            .withInvert(FEEDER_INVERT);

    }

    public static final class HoodMinLimitSwitchConstants {
        public static final String HOOD_MIN_LIMIT_SWITCH_NAME = "hood min Limit Switch";
        public static final int HOOD_MIN_LIMIT_SWITCH_ID = 9;
        public static final boolean HOOD_MIN_LIMIT_SWITCH_INVERT = true;
        public static final LimitSwitchConfig HOOD_MIN_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(HOOD_MIN_LIMIT_SWITCH_NAME,HOOD_MIN_LIMIT_SWITCH_ID)
            .withInvert(HOOD_MIN_LIMIT_SWITCH_INVERT);
    }

    public static enum ShooterStates implements MechanismState {
        SHOOTING,
        DELIVERY,
        GETTING_READY;

        @Override public double[] getValues() {
            return Shooter.getInstance().getShooterValues();
        }
    }
}
