package frc.robot.mechanismTesting.turret.commands;

import frc.demacia.utils.mechanisms.CalibrationCommand;
import frc.robot.mechanismTesting.turret.subsystems.Turret;
import frc.robot.mechanismTesting.turret.TurretConstants;

public class TurretCalibration extends CalibrationCommand{
    

    public TurretCalibration(Turret turret) {
        super(turret, TurretConstants.MOTOR_NAME, 0.1, () -> (turret.isAtMaxLimit() || turret.isAtMinLimit()), resetPosition(turret));
    }

    private static double resetPosition(Turret turret) {
        if (turret.isAtMinLimit()) {
            return TurretConstants.MIN_POSITION;
        } else if (turret.isAtMaxLimit()) {
            return TurretConstants.MAX_POSITION;
        } else {
            return 0;
        }
    }
}