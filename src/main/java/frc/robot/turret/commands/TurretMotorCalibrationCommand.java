package frc.robot.turret.commands;

import frc.demacia.utils.mechanisms.CalibrationCommand;
import frc.robot.turret.subsystems.Turret;
import static frc.robot.turret.TurretConstants.TurretMotorConstants.*;

public class TurretMotorCalibrationCommand extends CalibrationCommand {

    public TurretMotorCalibrationCommand(Turret mechanism) {
        super(
            mechanism, 
            TURRET_MOTOR_NAME, 
            TURRET_MOTOR_CALIBRATION_POWER,
            mechanism::atTurretMotorResetPos, 
            TURRET_MOTOR_CMD_CALIBRATION_RESET_POS 
        );
    }
}
