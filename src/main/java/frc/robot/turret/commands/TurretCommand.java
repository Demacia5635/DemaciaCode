package frc.robot.turret.commands;

import static frc.robot.turret.TurretConstants.*;
import static frc.robot.turret.TurretConstants.TurretMotorConstants.*;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.turret.subsystems.Turret;

public class TurretCommand extends DefaultCommand<TurretStates> {
    private final Turret turret = Turret.getInstance();
    
    public TurretCommand() {
        super(Turret.getInstance(), new ControlMode[] {
            ControlMode.DUTYCYCLE
        });
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (turret.getState()) {
            case SHOOTING, DELIVERY:
                turret.setTurretMotorPower(turret.getValue(TURRET_MOTOR_NAME));
                break;
            default:
                super.execute();
        }
    }
}
