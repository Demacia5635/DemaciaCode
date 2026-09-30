package frc.robot.shooter.commands;

import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.shooter.subsystems.Shooter;

public class ShooterCommand extends DefaultCommand {
    
    public ShooterCommand() {
        super(Shooter.getInstance(), new ControlMode[] {
            ControlMode.VELOCITY,
            ControlMode.MAGIC_MOTION,
            ControlMode.DUTYCYCLE
        });
    }
}
