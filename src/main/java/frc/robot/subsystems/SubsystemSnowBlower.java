package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstantsGripper;


public class SubsystemSnowBlower extends SubsystemBase {

    private final TalonSRX m_snowBlowerMotor;

    public SubsystemSnowBlower() {
        m_snowBlowerMotor = new TalonSRX(ConstantsGripper.kSnowblowerID);

    }

    public void setPower(double duty) {
        System.out.println("duty = " + duty);
        m_snowBlowerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, duty);
    }
    public void stop() {
        m_snowBlowerMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
    }
}