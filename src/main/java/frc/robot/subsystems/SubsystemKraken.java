package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import static frc.robot.constants.ConstantsGripper.*;

public class SubsystemKraken extends SubsystemBase {

    private final TalonFXMotor m_KRAKEN1 = new TalonFXMotor(CONFIG);
    private final TalonFXMotor m_KRAKEN2 = new TalonFXMotor(CONFIG2);

    public SubsystemKraken() {
        // Creates two input boxes in Elastic for Pitch and Roll (in degrees)
        SmartDashboard.putNumber("Differential Pitch (Degrees)", 0.0);
        SmartDashboard.putNumber("Differential Roll (Degrees)", 0.0);
    }

    public void setPosition(double pitchRadians, double rollRadians) {
        m_KRAKEN1.setPositionVoltage(pitchRadians + rollRadians);
        m_KRAKEN2.setPositionVoltage(pitchRadians - rollRadians);
    }

    public void stop() {
        m_KRAKEN1.stop();
        m_KRAKEN2.stop();
    }

    @Override
    public void periodic() {
        double pitchDegrees = SmartDashboard.getNumber("Differential Pitch (Degrees)", 0.0);
        double rollDegrees = SmartDashboard.getNumber("Differential Roll (Degrees)", 0.0);
        
        setPosition(Math.toRadians(pitchDegrees), Math.toRadians(rollDegrees));
    }
}
