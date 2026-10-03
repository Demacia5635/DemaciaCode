package frc.robot.shinua.commands;

import static frc.robot.shinua.ShinuaConstants.ShinuaRollersConstants.*;
import static frc.robot.shinua.ShinuaConstants.MecanumConstants.*;
import static frc.robot.shinua.ShinuaConstants.ShinuaStates.*;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.intake.subsystems.Intake;
import frc.robot.shinua.ShinuaConstants.ShinuaStates;
import frc.robot.shinua.subsystems.Shinua;
import frc.robot.shooter.subsystems.Shooter;
import frc.robot.turret.subsystems.Turret;

public class ShinuaCommand extends DefaultCommand<ShinuaStates> {
    private final Shinua shinua = Shinua.getInstance();
    
    public ShinuaCommand() {
        super(Shinua.getInstance(), new ControlMode[] {
            ControlMode.DUTYCYCLE,
            ControlMode.DUTYCYCLE
        });
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shinua.isStuckMecanum() || 
                shinua.isStuckShinuaRollers() || 
                Intake.getInstance().isStuckIntakeRollers() || 
                Shooter.getInstance().isStuckFeeder()) {
            shinua.setState(POOPING);
        }
 
        switch (shinua.getState()) {
            case SHOOTING:
                if (Shooter.getInstance().isShooterReady() && 
                        Turret.getInstance().isTurretReady() && 
                        Intake.getInstance().isIntakeReady()) {
                    shinua.setShinuaRollersPower(shinua.getValue(SHINUA_ROLLERS_NAME));
                    shinua.setMecanumPower(shinua.getValue(MECANUM_NAME));
                }
                break;
            case POOPING, NOT_SHOOTING:
                shinua.setShinuaRollersPower(shinua.getValue(SHINUA_ROLLERS_NAME));
                shinua.setMecanumPower(shinua.getValue(MECANUM_NAME));
                break;
            default:
                super.execute();
        }
    }
}
