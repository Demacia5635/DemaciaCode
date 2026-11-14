// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.CancoderConfig;

import frc.demacia.utils.motors.TalonFXConfig;

public final class IntakeConstants {

        // Adjust as needed
        public static final double DEPLOY_SPEED = 0.3;
        public static final double RETRACT_SPEED = -0.3;

        public static final double INTAKE_SPEED = 0.5;
        public static final double EJECT_SPEED = -0.5;

        public static final double ALIGNER_SPEED = 0.6;

        public static final double DEPLOYED_POSITION = 0.5; // Change
        public static final double RETRACTED_POSITION = 0.; // Change
        public static final double POSITION_TOLORANCE = 0.05; // Change

        public static final double TARGET_RETRACT_RADIANS = 0;

        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = true;
        public static final Canbus CANBUS = Canbus.Rio;
        public static final double MAX_CURRENT = 40;

        // Rollers motor config
        public static final int ROLLERS_MOTOR_ID = 1; // Change
        public static final String ROLLERS_MOTOR_NAME = "Rollers Motor";

        public static final TalonFXConfig ROLLERS_MOTOR_CONFIG = new TalonFXConfig(ROLLERS_MOTOR_ID, CANBUS,
                        ROLLERS_MOTOR_NAME)
                        .withBrake(IS_BRAKE)
                        .withInvert(IS_INVERTED)
                        .withCurrent(MAX_CURRENT);

        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

        // Deploy motor config
        public static final int DEPLOY_MOTOR_ID = 2; // Change
        public static final String DEPLOY_MOTOR_NAME = "Deploy Motor";

        public static final TalonFXConfig DEPLOY_MOTOR_CONFIG = new TalonFXConfig(DEPLOY_MOTOR_ID, CANBUS,
                        DEPLOY_MOTOR_NAME)
                        .withBrake(IS_BRAKE)
                        .withPID(KP, KI, KD, KS, KV, KA, KG)
                        .withInvert(IS_INVERTED)
                        .withCurrent(MAX_CURRENT);

        // Aligner motor config
        public static final int ALIGNER_MOTOR_ID = 3; // Change
        public static final String ALIGNER_MOTOR_NAME = "Aligner Motor";

        public static final TalonFXConfig ALIGNER_MOTOR_CONFIG = new TalonFXConfig(ALIGNER_MOTOR_ID, CANBUS,
                        ALIGNER_MOTOR_NAME)
                        .withBrake(IS_BRAKE)
                        .withInvert(IS_INVERTED)
                        .withCurrent(MAX_CURRENT);

        public static final int CANCODER_ID = 1; // Change
        public static final String CANCODER_NAME = "Cancoder";

        public static final CancoderConfig CANCODER_CONFIG = new CancoderConfig(CANCODER_ID, CANBUS, CANCODER_NAME);

        
        public enum IntakeState {
                IDLE, // both motors aren't working
                INTAKING, // rollers and align motors are moving and intaking the coral
                EJECTING, // rollers and align motors are moving and ejecting the coral
                HAS_CORAL; // the coral is secured inside the robot's perimeter
        }

        // if you need to add more (template):

        // public static final int TEMPLATE_MOTOR_ID = 2;
        // public static final Canbus TEMPLATE_MOTOR_CANBUS = Canbus.Rio;
        // public static final String TEMPLATE_MOTOR_NAME = "Template Motor";

        // public static final TalonFXConfig TEMPLATE_MOTOR_CONFIG = new
        // TalonFXConfig(TEMPLATE_MOTOR_ID, CANBUS, TEMPLATE_MOTOR_NAME)
        // .withBrake(IS_BRAKE);
}
