// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

import frc.demacia.utils.motors.TalonFXConfig;

public final class IntakeConstants {

        // Adjust as needed
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERTED = false;
        public static final Canbus CANBUS = Canbus.Rio;
        public static final double MAX_CURRENT = 40;
        public static final double MAX_VOLTS = 12;
        public static final double GEAR_RATIO = 1;

        // Rollers motor config
        public static final int ROLLERS_MOTOR_ID = 1; // Change
        public static final String ROLLERS_MOTOR_NAME = "Rollers Motor";

        public static final TalonFXConfig ROLLERS_MOTOR_CONFIG = new TalonFXConfig(ROLLERS_MOTOR_ID, CANBUS,
                        ROLLERS_MOTOR_NAME)
                        .withBrake(IS_BRAKE)
                        .withInvert(IS_INVERTED)
                        .withCurrent(MAX_CURRENT)
                        .withVolts(MAX_VOLTS);

        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KG = 0;

    private static final double MOTION_MAGIC_VELOCITY = 0;
    private static final double MOTION_MAGIC_ACCEL = 0;
    private static final double MOTION_MAGIC_JERK = 0;
        // Deploy motor config
        public static final int DEPLOY_MOTOR_ID = 2; // Change
        public static final String DEPLOY_MOTOR_NAME = "Deploy Motor";

        public static final TalonFXConfig DEPLOY_MOTOR_CONFIG = new TalonFXConfig(DEPLOY_MOTOR_ID, CANBUS,
                        DEPLOY_MOTOR_NAME)
                        .withBrake(IS_BRAKE)
                        .withPID(KP, KI, KD, KS, KV, KA, KG)
                        .withInvert(IS_INVERTED)
                        .withCurrent(MAX_CURRENT)
                        .withRadiansMotor(GEAR_RATIO)
                        .withMotionParam(MOTION_MAGIC_VELOCITY, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
                        .withVolts(MAX_VOLTS);

        // Aligner motor config
        public static final int ALIGNER_MOTOR_ID = 3; // Change
        public static final String ALIGNER_MOTOR_NAME = "Aligner Motor";

        public static final TalonFXConfig ALIGNER_MOTOR_CONFIG = new TalonFXConfig(ALIGNER_MOTOR_ID, CANBUS,
                        ALIGNER_MOTOR_NAME)
                        .withBrake(IS_BRAKE)
                        .withInvert(IS_INVERTED)
                        .withCurrent(MAX_CURRENT)
                        .withVolts(MAX_VOLTS);

        // Limit Switch
        public static final int LIMIT_SWITCH_ID = 1; // Change


        // Other
        public static final double DEPLOY_SPEED = 0.3;
        public static final double RETRACT_SPEED = -0.3;

        public static final double ROLLERS_SPEED = 0.5;

        public static final double ALIGNER_SPEED = 0.6;

        public static final double CLOSED_DEPLOY_ANGLE = Math.toRadians(0.0); // Change
        public static final double OPEN_DEPLOY_ANGLE = Math.toRadians(90.0); // Change
        public static final double ANGLE_TOLORANCE = Math.toRadians(5); // Change

        public static final double TARGET_RETRACT_RADIANS = 0;

        public enum IntakeState {

                INTAKING(ROLLERS_SPEED, ALIGNER_SPEED, OPEN_DEPLOY_ANGLE), // Deployed & rollers + aligner are on
                EJECTING(-ROLLERS_SPEED, -ALIGNER_SPEED, OPEN_DEPLOY_ANGLE), // Deployed and rollers + aligner are on but reversed (spitting the coral out)
                CLOSED(0, 0, CLOSED_DEPLOY_ANGLE), // Inside the robot, rollers + aligner are off
                DEPLOYED_IDLE(0,0,OPEN_DEPLOY_ANGLE), // Deployed, rollers + aligner are off
                IDLE(0, 0, 0), // Default state, nothing is on
                TESTING(0, 0, 0); // Testing (can change values in elastic)

                public final double rollersDuty;
                public final double deployAngle;
                public final double alignerDuty;

                IntakeState(double rollersDuty, double alignerDuty, double deployAngle) {
                        this.rollersDuty = rollersDuty;
                        this.deployAngle = deployAngle;
                        this.alignerDuty = alignerDuty;
                }
        }

        // if you need to add more (template):

        // public static final int TEMPLATE_MOTOR_ID = 2;
        // public static final Canbus TEMPLATE_MOTOR_CANBUS = Canbus.Rio;
        // public static final String TEMPLATE_MOTOR_NAME = "Template Motor";

        // public static final TalonFXConfig TEMPLATE_MOTOR_CONFIG = new
        // TalonFXConfig(TEMPLATE_MOTOR_ID, CANBUS, TEMPLATE_MOTOR_NAME)
        // .withBrake(IS_BRAKE);
}
