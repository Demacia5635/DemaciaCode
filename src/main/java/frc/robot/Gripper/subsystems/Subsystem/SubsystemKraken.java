package frc.robot.Gripper.subsystems.Subsystem;

import static frc.robot.Gripper.subsystems.constants.ConstantsGripper.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;

public class SubsystemKraken extends SubsystemBase {

    private final TalonFXMotor m_KRAKEN1Pitch;
    private final TalonFXMotor m_KRAKEN2Roll;
    private final LimitSwitch limitSwitchPitch;
    private final LimitSwitch limitSwitchRoll;

    public SubsystemKraken() {
        m_KRAKEN1Pitch = new TalonFXMotor(CONFIG);
        m_KRAKEN2Roll = new TalonFXMotor(CONFIG2);
        limitSwitchPitch = new LimitSwitch(LIMIT_SWITCH_CONFIG_PITCH);
        limitSwitchRoll = new LimitSwitch(LIMIT_SWITCH_CONFIG_ROLL);

        // Creates two input boxes in Elastic for Pitch and Roll (in degrees)
        SmartDashboard.putNumber("Differential Pitch (Degrees)", 0.0);
        SmartDashboard.putNumber("Differential Roll (Degrees)", 0.0);
    }

    public void setPosition(double pitchRadians, double rollRadians) {
        m_KRAKEN1Pitch.setPositionVoltage(pitchRadians + rollRadians);
        m_KRAKEN2Roll.setPositionVoltage(pitchRadians - rollRadians);
    }

    public boolean isPitchLimitSwitchTriggered() {
        return limitSwitchPitch.get();
    }

    public boolean isRollLimitSwitchTriggered() {
        return limitSwitchRoll.get();
    }

    public void stop() {
        m_KRAKEN1Pitch.stop();
        m_KRAKEN2Roll.stop();
    }

    public final double getPitchPosition() {
        return m_KRAKEN1Pitch.getPosition().getValueAsDouble();
    }

    public final double getRollPosition() {
        return m_KRAKEN2Roll.getPosition().getValueAsDouble();
    }

    public final double getPitchPositionDegrees() {
        return Math.toDegrees(getPitchPosition());
    }

    public final double getRollPositionDegrees() {
        return Math.toDegrees(getRollPosition());
    }

    public final void Is_Out_Max() {
        if (getPitchPositionDegrees() > 90 || getRollPositionDegrees() > 90) {
            stop();
        }
    }

    public void resetPitchEncoder() {
        m_KRAKEN1Pitch.getCurrentPosition();
    }

    public void Pitchmove(double speed) {
        m_KRAKEN1Pitch.setVoltage(speed);
        m_KRAKEN2Roll.setVoltage(speed);
    }

    public void rollmove(double speed) {
        m_KRAKEN1Pitch.setVoltage(speed);
        m_KRAKEN2Roll.setVoltage(-speed);
    }

    @Override
    public void periodic() {
        // Zero both encoders when both limit switches detect magnets
        if (isPitchLimitSwitchTriggered() && isRollLimitSwitchTriggered()) {
            m_KRAKEN1Pitch.setPosition(0);
            m_KRAKEN2Roll.setPosition(0);
        }

        // Read target angles from Elastic and clamp between -90 and 90 degrees
        double targetPitchDeg = MathUtil.clamp(
            SmartDashboard.getNumber("Differential Pitch (Degrees)", 0.0), -90.0, 90.0);
        double targetRollDeg = MathUtil.clamp(
            SmartDashboard.getNumber("Differential Roll (Degrees)", 0.0), -90.0, 90.0);

        // Move motors to target position
        setPosition(Math.toRadians(targetPitchDeg), Math.toRadians(targetRollDeg));

        // Calculate and display actual differential angles
        double motor1Rad = getPitchPosition();
        double motor2Rad = getRollPosition();
        double actualPitchDeg = Math.toDegrees((motor1Rad + motor2Rad) / 2.0);
        double actualRollDeg = Math.toDegrees((motor1Rad - motor2Rad) / 2.0);

        SmartDashboard.putNumber("Actual Pitch (Degrees)", actualPitchDeg);
        SmartDashboard.putNumber("Actual Roll (Degrees)", actualRollDeg);
    }
}
