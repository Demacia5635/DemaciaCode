package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.ConstantsGripper.*;

public class SubsystemKraken extends SubsystemBase {

    private final TalonFX m_KRAKEN1 = new TalonFX(kKrakenID, kSKrakencanbus.toString());
    private final TalonFX m_KRAKEN2 = new TalonFX(kKrakenID2, kSKrakencanbus2.toString());

    public SubsystemKraken() {
      
    }

    public void setPowerforwadandback(double speed) { // TODO: change to CAPS LETER
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
    public void position1(){
    }

    @Override
    public void periodic() {
    }
}