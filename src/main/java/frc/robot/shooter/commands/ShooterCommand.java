package frc.robot.shooter.commands;

import static frc.robot.shooter.ShooterConstants.FlywheelConstants.*;
import static frc.robot.shooter.ShooterConstants.HoodConstants.*;
import static frc.robot.shooter.ShooterConstants.FeederConstants.*;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.intake.subsystems.Intake;
import frc.robot.shinua.subsystems.Shinua;
import frc.robot.shooter.ShooterConstants.ShooterStates;
import frc.robot.shooter.subsystems.Shooter;

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
        switch (shooter.getState()) {
            case SHOOTING, DELIVERY, TRANCH:
                shooter.setFlywheelVelocity(shooter.getValue(FLYWHEEL_NAME));
                shooter.setHoodMotion(shooter.getValue(HOOD_NAME));
                if (Shinua.getInstance().isStuckMecanum() || 
                        Shinua.getInstance().isStuckShinuaRollers() || 
                        Intake.getInstance().isStuckIntakeRollers() || 
                        shooter.isStuckFeeder()) {
                    shooter.setFeederPower(-1);
                } else {
                    shooter.setFeederPower(shooter.getValue(FEEDER_NAME));
                }
                break;
            default:
                super.execute();
        }
    }
}
