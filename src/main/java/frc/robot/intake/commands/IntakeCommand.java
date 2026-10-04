package frc.robot.intake.commands;

import static frc.robot.intake.IntakeConstants.IntakeRollersConstants.*;
import static frc.robot.intake.IntakeConstants.IntakeDeployConstants.*;
import static frc.robot.intake.IntakeConstants.IntakeStates.*;

import frc.demacia.utils.log.Log;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.intake.IntakeConstants.IntakeStates;
import frc.robot.intake.subsystems.Intake;
import frc.robot.shinua.subsystems.Shinua;
import frc.robot.shooter.subsystems.Shooter;

public class IntakeCommand extends DefaultCommand<IntakeStates> {
    private final Intake intake = Intake.getInstance();
    
    public IntakeCommand() {
        super(Intake.getInstance(), new ControlMode[] {
            ControlMode.DUTYCYCLE,
            ControlMode.MAGIC_MOTION
        });
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Shinua.getInstance().isStuckMecanum() || 
                Shinua.getInstance().isStuckShinuaRollers() || 
                intake.isStuckIntakeRollers() || 
                Shooter.getInstance().isStuckFeeder()) {
                    Log.alert(Shinua.getInstance().isStuckMecanum() + "isStuckMecanum");
                    Log.alert(Shinua.getInstance().isStuckShinuaRollers() + "isStuckShinuaRollers");
                    Log.alert(intake.isStuckIntakeRollers() + "isStuckIntakeRollers");
                    Log.alert(Shooter.getInstance().isStuckFeeder() + "isStuckFeeder");
                    intake.setState(POOPING);
        }

        //if state is testing or idle then getState() is null and DefaultCommand execute will handle it
        if (intake.getState() == null) {
            super.execute();
            return;
        }

        switch (intake.getState()) {
            case INTAKING, POOPING, CLOSED, MIDDLE, SHOOTING:
                intake.setIntakeRollersPower(intake.getValue(INTAKE_ROLLERS_NAME));
                intake.setIntakeDeployMotion(intake.getValue(INTAKE_DEPLOY_NAME));
                break;
            default:
                break;
        }
    }
}
