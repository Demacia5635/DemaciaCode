package frc.robot.shinua;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;

public class ShinuaConstants {
    public static final String SHINUA_NAME = "shinua";

    public static final class MechanomConstants {
        public static final String MECHANOM_NAME = "mechanom";
        public static final int MECHANOM_ID = 40;
        public static final Canbus MECHANOM_CANBUS = Canbus.Rio; // TODO
        public static final boolean MECHANOM_BRAKE = false;
        public static final boolean MECHANOM_INVERT = false; // TODO

        public static final TalonFXConfig MECHANOM_CONFIG = new TalonFXConfig(MECHANOM_NAME, MECHANOM_ID, MECHANOM_CANBUS)
            .withBrake(MECHANOM_BRAKE)
            .withInvert(MECHANOM_INVERT);

    }

    public static final class ShinuaRollersConstants {
        public static final String SHINUA_ROLLERS_NAME = "shinua rollers";
        public static final int SHINUA_ROLLERS_ID = 41;
        public static final Canbus SHINUA_ROLLERS_CANBUS = Canbus.Rio; // TODO
        public static final boolean SHINUA_ROLLERS_BRAKE = false;
        public static final boolean SHINUA_ROLLERS_INVERT = false; // TODO

        public static final TalonFXConfig SHINUA_ROLLERS_CONFIG = new TalonFXConfig(SHINUA_ROLLERS_NAME, SHINUA_ROLLERS_ID, SHINUA_ROLLERS_CANBUS)
            .withBrake(SHINUA_ROLLERS_BRAKE)
            .withInvert(SHINUA_ROLLERS_INVERT);

    }

    public static enum ShinuaStates implements MechanismState {
        ON(1.0, -0.1), // TODO
        OFF(0.0, -0.1); // TODO

        private final double[] values;
        private ShinuaStates(double... vals) { this.values = vals; }
        @Override public double[] getValues() { return values; }
    }
}
