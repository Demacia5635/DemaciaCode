package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ConstantsGripper.*;

public class SubsystemKraken extends SubsystemBase {

    private final TalonFX m_KRAKEN1 = new TalonFX(kKrakenID, kSKrakencanbus.toString());
    private final TalonFX m_KRAKEN2 = new TalonFX(kKrakenID2, kSKrakencanbus2.toString());

    // Phoenix 6 control request configuration for position tracking
    private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);

    public SubsystemKraken() {
        // Initialize the network table keys with default values so they show up instantly in Elastic
        SmartDashboard.putNumber("Kraken Target Position", 0.0);
        SmartDashboard.putNumber("Kraken1 Real Position", 0.0);
        SmartDashboard.putNumber("Kraken2 Real Position", 0.0);

        // Optional: Configure your PID gains in slot 0 if not already done in robot initialization
        /*
        var tuner = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        tuner.Slot0.kP = 2.4; // Replace with your tuned P value
        tuner.Slot0.kI = 0.0;
        tuner.Slot0.kD = 0.1; // Replace with your tuned D value
        m_KRAKEN1.getConfigurator().apply(tuner);
        m_KRAKEN2.getConfigurator().apply(tuner);
        */
    }

    /**
     * Drives both motors to a specific closed-loop position target (rotations).
     * @param targetRotations Target position measured in motor rotations.
     */
    public void setPosition(double targetRotations) {
        // Send the control request to the TalonFX internal PID loop
        m_KRAKEN1.setControl(m_positionRequest.withPosition(targetRotations));
        m_KRAKEN2.setControl(m_positionRequest.withPosition(targetRotations));
    }

    public void setPowerforwadandback(double speed) { 
        m_KRAKEN1.set(speed);
        m_KRAKEN2.set(speed);
    }
    
    public void setpowerturnleftandright(double speed) {
        m_KRAKEN1.set(speed);
        m_KRAKEN2.set(-speed);
    }

    public void stop() {
        m_KRAKEN1.stopMotor();
        m_KRAKEN2.stopMotor();
    }

    @Override
    public void periodic() {
        // 1. Read the target position that you or another teammate typed into the Elastic Dashboard
        double dashboardTarget = SmartDashboard.getNumber("Demacia/Kraken Target Position", 0.0);
        
        // 2. Continually apply that position targets to your Krakens
        setPosition(dashboardTarget);

        // 3. Post telemetry back to Elastic so you can verify the hardware matches the command
        SmartDashboard.putNumber("Demacia/Kraken1 Real Position", m_KRAKEN1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Demacia/Kraken2 Real Position", m_KRAKEN2.getPosition().getValueAsDouble());
    }
}
