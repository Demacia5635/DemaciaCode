package frc.robot.intake.commands;

import frc.demacia.utils.mechanisms.CalibrationCommand;
import frc.robot.intake.subsystems.Intake;
import static frc.robot.intake.IntakeConstants.IntakeDeployConstants.*;

public class IntakeDeployCalibrationCommand extends CalibrationCommand {

    public IntakeDeployCalibrationCommand(Intake mechanism) {
        super(
            mechanism, 
            INTAKE_DEPLOY_NAME, 
            INTAKE_DEPLOY_CALIBRATION_POWER,
            mechanism::atIntakeDeployResetPos, 
            INTAKE_DEPLOY_CMD_CALIBRATION_RESET_POS 
        );
    }
}
