package frc.robot.shooter.commands;

import static frc.robot.shooter.ShooterConstants.FlywheelConstants.*;
import static frc.robot.shooter.ShooterConstants.HoodConstants.*;
import static frc.robot.shooter.ShooterConstants.FeederConstants.*;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.shinua.subsystems.Shinua;
import frc.robot.shooter.ShooterConstants.ShooterStates;
import frc.robot.shooter.subsystems.Shooter;
import frc.robot.turret.subsystems.Turret;

public class ShooterCommand extends DefaultCommand<ShooterStates> {
    private final Shooter shooter = Shooter.getInstance();
    
    public ShooterCommand() {
        super(Shooter.getInstance(), new ControlMode[] {
            ControlMode.VELOCITY,
            ControlMode.MAGIC_MOTION,
            ControlMode.DUTYCYCLE
        });
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //if state is testing or idle getState() is null and DefaultCommand will handle it
        if (shooter.getState() == null) {
            super.execute();
            return;
        }

        switch (shooter.getState()) {
            case SHOOTING, DELIVERY, TRANCH:
                shooter.setFlywheelVelocity(shooter.getValue(FLYWHEEL_NAME));
                shooter.setHoodMotion(shooter.getValue(HOOD_NAME));
                if (Shinua.getInstance().isStuckMecanum() || 
                        Shinua.getInstance().isStuckShinuaRollers() || 
                        shooter.isStuckFeeder()) {
                    shooter.setFeederPower(FEEDER_POOPING_POWER);
                } else if (Shooter.getInstance().isShooterReady() && 
                        Turret.getInstance().isTurretReady()) {
                    shooter.setFeederPower(0);
                } else {
                    shooter.setFeederPower(shooter.getValue(FEEDER_NAME));
                }
                break;
            default:
                break;
        }
    }
}
