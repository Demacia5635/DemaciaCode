package frc.robot.shinua;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;

public class ShinuaConstants {
    public static final String SHINUA_NAME = "shinua";

    public static final class ShinuaRollersConstants {
        public static final String SHINUA_ROLLERS_NAME = "shinua rollers";
        public static final int SHINUA_ROLLERS_ID = 40;
        public static final Canbus SHINUA_ROLLERS_CANBUS = Canbus.Rio; // TODO
        public static final boolean SHINUA_ROLLERS_BRAKE = false;
        public static final boolean SHINUA_ROLLERS_INVERT = false;
        public static final double SHINUA_ROLLERS_HIGH_CURRENT_THRESHOLD = 30.0; // TODO
        public static final double SHINUA_ROLLERS_LOW_VELOCITY_THRESHOLD = 5.0; // TODO
        public static final double SHINUA_ROLLERS_STALL_CONFIRM_SECONDS = 0.2; // TODO
        public static final double SHINUA_ROLLERS_STUCK_DURATION_SECONDS = 0.2; // TODO

        public static final TalonFXConfig SHINUA_ROLLERS_CONFIG = new TalonFXConfig(SHINUA_ROLLERS_NAME, SHINUA_ROLLERS_ID, SHINUA_ROLLERS_CANBUS)
            .withBrake(SHINUA_ROLLERS_BRAKE)
            .withInvert(SHINUA_ROLLERS_INVERT)
            .withDetectStall(SHINUA_ROLLERS_HIGH_CURRENT_THRESHOLD, SHINUA_ROLLERS_LOW_VELOCITY_THRESHOLD, SHINUA_ROLLERS_STALL_CONFIRM_SECONDS, SHINUA_ROLLERS_STUCK_DURATION_SECONDS);

    }

    public static final class MecanumConstants {
        public static final String MECANUM_NAME = "mecanum";
        public static final int MECANUM_ID = 41;
        public static final Canbus MECANUM_CANBUS = Canbus.Rio; // TODO
        public static final boolean MECANUM_BRAKE = false;
        public static final boolean MECANUM_INVERT = false;
        public static final double MECANUM_HIGH_CURRENT_THRESHOLD = 30.0; // TODO
        public static final double MECANUM_LOW_VELOCITY_THRESHOLD = 5.0; // TODO
        public static final double MECANUM_STALL_CONFIRM_SECONDS = 0.2; // TODO
        public static final double MECANUM_STUCK_DURATION_SECONDS = 0.2; // TODO

        public static final TalonFXConfig MECANUM_CONFIG = new TalonFXConfig(MECANUM_NAME, MECANUM_ID, MECANUM_CANBUS)
            .withBrake(MECANUM_BRAKE)
            .withInvert(MECANUM_INVERT)
            .withDetectStall(MECANUM_HIGH_CURRENT_THRESHOLD, MECANUM_LOW_VELOCITY_THRESHOLD, MECANUM_STALL_CONFIRM_SECONDS, MECANUM_STUCK_DURATION_SECONDS);

    }

    public static enum ShinuaStates implements MechanismState {
        SHOOTING(-0.1, 0.7), // TODO
        POOPING(-0.3, 0.1), // TODO
        NOT_SHOOTING(-0.3, 0.1), 
        NO_MECANUM(-0.05, 0); // TODO

        private final double[] values;
        private ShinuaStates(double... vals) { this.values = vals; }
        @Override public double[] getValues() { return values; }
    }
}
