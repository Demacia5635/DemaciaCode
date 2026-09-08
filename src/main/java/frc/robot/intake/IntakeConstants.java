package frc.robot.intake;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.LimitSwitchConfig;

public class IntakeConstants {
    public static final String INTAKE_NAME = "intake";

    public static final class IntakeRollerConstants {
        public static final String INTAKE_ROLLER_NAME = "intake roller";
        public static final int INTAKE_ROLLER_ID = 30;
        public static final Canbus INTAKE_ROLLER_CANBUS = Canbus.Rio; // TODO
        public static final boolean INTAKE_ROLLER_BRAKE = false;
        public static final boolean INTAKE_ROLLER_INVERT = false; // TODO

        public static final TalonFXConfig INTAKE_ROLLER_CONFIG = new TalonFXConfig(INTAKE_ROLLER_NAME, INTAKE_ROLLER_ID, INTAKE_ROLLER_CANBUS)
            .withBrake(INTAKE_ROLLER_BRAKE)
            .withInvert(INTAKE_ROLLER_INVERT);

    }

    public static final class IntakeDeployConstants {
        public static final String INTAKE_DEPLOY_NAME = "intake deploy";
        public static final int INTAKE_DEPLOY_ID = 31;
        public static final Canbus INTAKE_DEPLOY_CANBUS = Canbus.Rio; // TODO
        public static final boolean INTAKE_DEPLOY_BRAKE = true;
        public static final boolean INTAKE_DEPLOY_INVERT = false;
        public static final double INTAKE_DEPLOY_GEAR_RATIO = 64.0;
        public static final double INTAKE_DEPLOY_KP = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KI = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KD = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KS = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KV = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KA = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KG = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KV2 = 0.0; // TODO
        public static final double INTAKE_DEPLOY_KCOS = 0.0; // TODO

        public static final TalonFXConfig INTAKE_DEPLOY_CONFIG = new TalonFXConfig(INTAKE_DEPLOY_NAME, INTAKE_DEPLOY_ID, INTAKE_DEPLOY_CANBUS)
            .withBrake(INTAKE_DEPLOY_BRAKE)
            .withInvert(INTAKE_DEPLOY_INVERT)
            .withRadiansMotor(INTAKE_DEPLOY_GEAR_RATIO)
            .withPID(INTAKE_DEPLOY_KP, INTAKE_DEPLOY_KI, INTAKE_DEPLOY_KD, INTAKE_DEPLOY_KS, INTAKE_DEPLOY_KV, INTAKE_DEPLOY_KA, INTAKE_DEPLOY_KG, INTAKE_DEPLOY_KCOS, INTAKE_DEPLOY_KV2);

        public static final double INTAKE_DEPLOY_MIN_LIMIT = 0.0; // TODO
        public static final double INTAKE_DEPLOY_MAX_LIMIT = 0.0; // TODO
        public static final double INTAKE_DEPLOY_CALIBRATION_POWER = 0.2;
        public static final double INTAKE_DEPLOY_CMD_CALIBRATION_RESET_POS = 0.0; // TODO
        public static final double INTAKE_DEPLOY_AUTO_CALIBRATION_RESET_POS = 0.0; // TODO
    }

    public static final class IntakeDeployMaxLimitSwitchConstants {
        public static final String INTAKE_DEPLOY_MAX_LIMIT_SWITCH_NAME = "intake deploy max Limit Switch";
        public static final int INTAKE_DEPLOY_MAX_LIMIT_SWITCH_ID = 7;
        public static final boolean INTAKE_DEPLOY_MAX_LIMIT_SWITCH_INVERT = true; // TODO
        public static final LimitSwitchConfig INTAKE_DEPLOY_MAX_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_NAME,INTAKE_DEPLOY_MAX_LIMIT_SWITCH_ID)
            .withInvert(INTAKE_DEPLOY_MAX_LIMIT_SWITCH_INVERT);
    }

    public static enum IntakeStates implements MechanismState {
        CLOSED(0.0, 0.0), // TODO
        INTAKING(1.0, 0.0), // TODO
        POOPING(-1.0, 0.0); // TODO

        private final double[] values;
        private IntakeStates(double... vals) { this.values = vals; }
        @Override public double[] getValues() { return values; }
    }
}
