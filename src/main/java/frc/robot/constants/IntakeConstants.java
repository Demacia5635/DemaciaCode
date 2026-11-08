// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;


public final class IntakeConstants {


    // Adjust as needed
    public static final double DEPLOY_SPEED = 0.3;
    public static final double RETRACT_SPEED = -0.3;

    public static final double INTAKE_SPEED = 0.5;
    public static final double EJECT_SPEED = -0.5;


    public static final boolean IS_BRAKE = true; // Right now I use the same IS_BREAK for all motors but it can be changed if needed
    public static final Canbus CANBUS = Canbus.Rio;


    public static final int ROLLERS_MOTOR_ID = 1; // Change
    public static final String ROLLERS_MOTOR_NAME = "Rollers Motor";

    public static final TalonFXConfig ROLLERS_MOTOR_CONFIG = new TalonFXConfig(ROLLERS_MOTOR_ID, CANBUS, ROLLERS_MOTOR_NAME)
            .withBrake(IS_BRAKE);



    public static final int DEPLOY_MOTOR_ID = 2; // Change
    public static final Canbus DEPLOY_MOTOR_CANBUS = Canbus.Rio;
    public static final String DEPLOY_MOTOR_NAME = "Deploy Motor";

    public static final TalonFXConfig DEPLOY_MOTOR_CONFIG = new TalonFXConfig(DEPLOY_MOTOR_ID, CANBUS, DEPLOY_MOTOR_NAME)
            .withBrake(IS_BRAKE);


        


    // public static final int TEMPLATE_MOTOR_ID = 2;
    // public static final Canbus TEMPLATE_MOTOR_CANBUS = Canbus.Rio;
    // public static final String TEMPLATE_MOTOR_NAME = "Template Motor";

    // public static final TalonFXConfig TEMPLATE_MOTOR_CONFIG = new TalonFXConfig(TEMPLATE_MOTOR_ID, CANBUS, TEMPLATE_MOTOR_NAME)
    //         .withBrake(IS_BRAKE);
}
