package frc.robot.intake.commands;

import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.intake.subsystems.Intake;

public class IntakeCommand extends DefaultCommand {
    
    public IntakeCommand() {
        super(Intake.getInstance(), new ControlMode[] {
            ControlMode.DUTYCYCLE,
            ControlMode.ANGLE
        });
    }
}
