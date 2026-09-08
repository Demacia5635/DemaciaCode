package frc.robot.shooter.commands;

import frc.demacia.utils.mechanisms.CalibrationCommand;
import frc.robot.shooter.subsystems.Shooter;
import static frc.robot.shooter.ShooterConstants.HoodConstants.*;

public class HoodCalibrationCommand extends CalibrationCommand {

    public HoodCalibrationCommand(Shooter mechanism) {
        super(
            mechanism, 
            HOOD_NAME, 
            HOOD_CALIBRATION_POWER,
            mechanism::atHoodResetPos, 
            HOOD_CMD_CALIBRATION_RESET_POS 
        );
    }
}
