package frc.robot.intake.commands;

import static frc.robot.intake.IntakeConstants.IntakeDeployConstants.*;
import static frc.robot.intake.IntakeConstants.IntakeRollersConstants.*;
import frc.demacia.utils.mechanisms.DefaultCommand;
import frc.demacia.utils.motors.MotorInterface.ControlMode;
import frc.robot.intake.IntakeConstants.IntakeStates;
import frc.robot.intake.subsystems.Intake;

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
        switch (intake.getState()) {
            case INTAKING, POOPING:
                intake.setRollersPower(intake.getValue(INTAKE_ROLLERS_NAME));
                intake.setIntakeDeployMotion(intake.getValue(INTAKE_DEPLOY_NAME));
                break;
            default:
                super.execute();
        }
    }
}
