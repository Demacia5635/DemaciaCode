package frc.robot.shootet;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.sensors.CancoderConfig;

public class ShootetConstants {
    public static final String SHOOTET_NAME = "shootet";

    public static final class UnnamedMotor1Constants {
        public static final String UNNAMED_MOTOR_1_NAME = "Unnamed Motor 1";
        public static final int UNNAMED_MOTOR_1_ID = 0;
        public static final Canbus UNNAMED_MOTOR_1_CANBUS = Canbus.Rio;
        public static final boolean UNNAMED_MOTOR_1_BRAKE = true;
        public static final boolean UNNAMED_MOTOR_1_INVERT = false;

        public static final TalonFXConfig UNNAMED_MOTOR_1_CONFIG = new TalonFXConfig(UNNAMED_MOTOR_1_ID, UNNAMED_MOTOR_1_CANBUS, UNNAMED_MOTOR_1_NAME)
            .withBrake(UNNAMED_MOTOR_1_BRAKE)
            .withInvert(UNNAMED_MOTOR_1_INVERT);

    }

    public static final class HoodConstants {
        public static final String HOOD_NAME = "hood";
        public static final int HOOD_ID = 0;
        public static final Canbus HOOD_CANBUS = Canbus.Rio;
        public static final boolean HOOD_BRAKE = true;
        public static final boolean HOOD_INVERT = false;

        public static final TalonFXConfig HOOD_CONFIG = new TalonFXConfig(HOOD_ID, HOOD_CANBUS, HOOD_NAME)
            .withBrake(HOOD_BRAKE)
            .withInvert(HOOD_INVERT);

    }

    public static final class UnnamedMotor3Constants {
        public static final String UNNAMED_MOTOR_3_NAME = "Unnamed Motor 3";
        public static final int UNNAMED_MOTOR_3_ID = 0;
        public static final Canbus UNNAMED_MOTOR_3_CANBUS = Canbus.Rio;
        public static final boolean UNNAMED_MOTOR_3_BRAKE = true;
        public static final boolean UNNAMED_MOTOR_3_INVERT = false;

        public static final TalonFXConfig UNNAMED_MOTOR_3_CONFIG = new TalonFXConfig(UNNAMED_MOTOR_3_ID, UNNAMED_MOTOR_3_CANBUS, UNNAMED_MOTOR_3_NAME)
            .withBrake(UNNAMED_MOTOR_3_BRAKE)
            .withInvert(UNNAMED_MOTOR_3_INVERT);

    }

    public static final class UnnamedSensor1CancoderConstants {
        public static final String UNNAMED_SENSOR_1_CANCODER_NAME = "Unnamed Sensor 1 Cancoder";
        public static final int UNNAMED_SENSOR_1_CANCODER_ID = 0;
        public static final Canbus UNNAMED_SENSOR_1_CANCODER_CANBUS = Canbus.Rio;
        public static final boolean UNNAMED_SENSOR_1_CANCODER_INVERT = false;
        public static final CancoderConfig UNNAMED_SENSOR_1_CANCODER_CONFIG = new CancoderConfig(UNNAMED_SENSOR_1_CANCODER_ID, UNNAMED_SENSOR_1_CANCODER_CANBUS, UNNAMED_SENSOR_1_CANCODER_NAME)
            .withInvert(UNNAMED_SENSOR_1_CANCODER_INVERT);
    }

}
