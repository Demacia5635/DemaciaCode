package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ConstantsGripper.*;

public class SubsystemSnowBlower extends SubsystemBase {

    private final TalonFX m_snowBlowerMotor = new TalonFX(kSnowblowerID, kSnowBlowerCanbus.toString());

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