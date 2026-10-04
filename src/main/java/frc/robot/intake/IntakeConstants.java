package frc.robot.intake;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class IntakeConstants {
    public static final String INTAKE_NAME = "intake";

    public static final class IntakeRollersConstants {
        public static final String INTAKE_ROLLERS_NAME = "intake rollers";
        public static final int INTAKE_ROLLERS_ID = 51;
        public static final Canbus INTAKE_ROLLERS_CANBUS = Canbus.Rio; // TODO
        public static final boolean INTAKE_ROLLERS_BRAKE = false;
        public static final boolean INTAKE_ROLLERS_INVERT = true;
        public static final double INTAKE_ROLLERS_HIGH_CURRENT_THRESHOLD = 30; // TODO
        public static final double INTAKE_ROLLERS_LOW_VELOCITY_THRESHOLD = 5; // TODO
        public static final double INTAKE_ROLLERS_STALL_CONFIRM_SECONDS = 0.2; // TODO
        public static final double INTAKE_ROLLERS_STUCK_DURATION_SECONDS = 0.2; // TODO

        public static final TalonFXConfig INTAKE_ROLLERS_CONFIG = new TalonFXConfig(INTAKE_ROLLERS_NAME, INTAKE_ROLLERS_ID, INTAKE_ROLLERS_CANBUS)
            .withBrake(INTAKE_ROLLERS_BRAKE)
            .withInvert(INTAKE_ROLLERS_INVERT)
            .withDetectStall(INTAKE_ROLLERS_HIGH_CURRENT_THRESHOLD, INTAKE_ROLLERS_LOW_VELOCITY_THRESHOLD, INTAKE_ROLLERS_STALL_CONFIRM_SECONDS, INTAKE_ROLLERS_STUCK_DURATION_SECONDS);

    }

    public static final class IntakeDeployConstants {
        public static final String INTAKE_DEPLOY_NAME = "intake deploy";
        public static final int INTAKE_DEPLOY_ID = 50;
        public static final Canbus INTAKE_DEPLOY_CANBUS = Canbus.Rio; // TODO
        public static final boolean INTAKE_DEPLOY_BRAKE = true;
        public static final boolean INTAKE_DEPLOY_INVERT = false;
        public static final double INTAKE_DEPLOY_GEAR_RATIO = 64;
        public static final double INTAKE_DEPLOY_KP = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KI = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KD = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KS = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KV = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KA = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KG = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KV2 = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KCOS = 0.0; // TODO
        public static final double INTAKE_DEPLOY_MAX_VELOCITY = 0.0; // TODO
        public static final double INTAKE_DEPLOY_MAX_ACCELERATION = 0.0; // TODO
        public static final double INTAKE_DEPLOY_MAX_JERK = 0.0; // TODO

        public static final TalonFXConfig INTAKE_DEPLOY_CONFIG = new TalonFXConfig(INTAKE_DEPLOY_NAME, INTAKE_DEPLOY_ID, INTAKE_DEPLOY_CANBUS)
            .withBrake(INTAKE_DEPLOY_BRAKE)
            .withInvert(INTAKE_DEPLOY_INVERT)
            .withRadiansMotor(INTAKE_DEPLOY_GEAR_RATIO)
            .withPID(INTAKE_DEPLOY_KP, INTAKE_DEPLOY_KI, INTAKE_DEPLOY_KD, INTAKE_DEPLOY_KS, INTAKE_DEPLOY_KV, INTAKE_DEPLOY_KA, INTAKE_DEPLOY_KG, INTAKE_DEPLOY_KCOS, INTAKE_DEPLOY_KV2)
            .withMotionParam(INTAKE_DEPLOY_MAX_VELOCITY, INTAKE_DEPLOY_MAX_ACCELERATION, INTAKE_DEPLOY_MAX_JERK);

        public static final double INTAKE_DEPLOY_MIN_LIMIT = Math.toRadians(-10); // TODO
        public static final double INTAKE_DEPLOY_MAX_LIMIT = Math.toRadians(100); // TODO
        public static final double INTAKE_DEPLOY_ALLOWED_POS = Math.toRadians(45); // TODO
        public static final double INTAKE_DEPLOY_CALIBRATION_POWER = 0.2;
        public static final double INTAKE_DEPLOY_CMD_CALIBRATION_RESET_POS = Math.toRadians(100);
        public static final double INTAKE_DEPLOY_AUTO_CALIBRATION_RESET_POS = Math.toRadians(100);

        public static final double INTAKE_DEPLOY_CLOSED = Math.toRadians(100);
        public static final double INTAKE_DEPLOY_MIDDLE = Math.toRadians(40);
        public static final double INTAKE_DEPLOY_OPEN = Math.toRadians(0);
    }

    public static final class IntakeDeployMaxLimitSwitchConstants {
        public static final String INTAKE_DEPLOY_MAX_LIMIT_SWITCH_NAME = "intake deploy max Limit Switch";
        public static final int INTAKE_DEPLOY_MAX_LIMIT_SWITCH_ID = 7; // TODO
        public static final boolean INTAKE_DEPLOY_MAX_LIMIT_SWITCH_INVERT = true;
        public static final LimitSwitchConfig INTAKE_DEPLOY_MAX_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_NAME,INTAKE_DEPLOY_MAX_LIMIT_SWITCH_ID)
            .withInvert(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_INVERT);
    }

    public static enum IntakeStates implements MechanismState {
        INTAKING(0.7, IntakeDeployConstants.INTAKE_DEPLOY_OPEN), // TODO
        POOPING(-0.7, IntakeDeployConstants.INTAKE_DEPLOY_OPEN), // TODO
        CLOSED(0.0, IntakeDeployConstants.INTAKE_DEPLOY_CLOSED), // TODO
        MIDDLE(0.0, IntakeDeployConstants.INTAKE_DEPLOY_MIDDLE), // TODO
        SHOOTING(0.2, IntakeDeployConstants.INTAKE_DEPLOY_MIDDLE); // TODO

        private final double[] values;
        private IntakeStates(double... vals) { this.values = vals; }
        @Override public double[] getValues() { return values; }
    }
}
