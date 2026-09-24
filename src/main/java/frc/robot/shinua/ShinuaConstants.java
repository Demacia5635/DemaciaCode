package frc.robot.shinua;

import frc.demacia.utils.mechanisms.StateBaseMechanism.MechanismState;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;

public class ShinuaConstants {
    public static final String SHINUA_NAME = "shinua";

    public static final class RollersConstants {
        public static final String ROLLERS_NAME = "rollers";
        public static final int ROLLERS_ID = 0; // TODO
        public static final Canbus ROLLERS_CANBUS = Canbus.Rio; // TODO
        public static final boolean ROLLERS_BRAKE = false;
        public static final boolean ROLLERS_INVERT = false;

        public static final TalonFXConfig ROLLERS_CONFIG = new TalonFXConfig(ROLLERS_NAME, ROLLERS_ID, ROLLERS_CANBUS)
            .withBrake(ROLLERS_BRAKE)
            .withInvert(ROLLERS_INVERT);

    }

    public static final class MechanomConstants {
        public static final String MECHANOM_NAME = "mechanom";
        public static final int MECHANOM_ID = 0; // TODO
        public static final Canbus MECHANOM_CANBUS = Canbus.Rio; // TODO
        public static final boolean MECHANOM_BRAKE = false;
        public static final boolean MECHANOM_INVERT = false;

        public static final TalonFXConfig MECHANOM_CONFIG = new TalonFXConfig(MECHANOM_NAME, MECHANOM_ID, MECHANOM_CANBUS)
            .withBrake(MECHANOM_BRAKE)
            .withInvert(MECHANOM_INVERT);

    }

    public static enum ShinuaStates implements MechanismState {
        INTAKING(-0.1, 1.0), // TODO
        WAITING(-0.4, 0.0), // TODO
        POOPING(-0.4, 0.1); // TODO

        private final double[] values;
        private ShinuaStates(double... vals) { this.values = vals; }
        @Override public double[] getValues() { return values; }
    }
}
