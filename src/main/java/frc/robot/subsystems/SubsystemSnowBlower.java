package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.constants.ConstantsGripper;

import static frc.robot.constants.ConstantsGripper.*;

public class SubsystemSnowBlower extends SubsystemBase {

    private final TalonFXMotor m_snowBlowerMotor = new TalonFXMotor(ConstantsGripper.kSnowBlowerConfig);

    public SubsystemSnowBlower() {
      
    }

    public void setPower(double speed) {
        m_snowBlowerMotor.set(speed);
    }

    public void stop() {
        m_snowBlowerMotor.stopMotor();
    }

    @Override
    public void periodic() {
    }
}