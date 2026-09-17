package frc.robot.turret.commands;

import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.turret.subsystems.Turret;

public class TurretCommand extends DefaultCommand {
    
    public TurretCommand() {
        super(Turret.getInstance(), new ControlMode[] {
            ControlMode.ANGLE
        });
    }
}
