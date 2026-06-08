package frc.robot.mechanismTesting.turret.commands;

import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.mechanismTesting.turret.subsystems.Turret;

public class TurretCommand extends DefaultCommand{
    

    public TurretCommand(Turret turret) {
        super(turret, new ControlMode[] {ControlMode.POSITION_VOLTAGE});
    }
}
