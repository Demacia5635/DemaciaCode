package frc.robot.constants;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;
import frc.demacia.utils.sensors.OpticalSensorConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

public class ConstantsGripper {
    public static final int kSnowblowerID = 32;
    public static final String kSnowBlowerName = "Snow_Blower";
    
    public static final TalonSRXConfig kSnowBlowerConfig = new TalonSRXConfig(
        kSnowblowerID,
        kSnowBlowerName
    );
    public static final int K_OPTIC_SENSOR_ID = 8;
    public static final String kOpticSensorNAME = "Optic_Sensor"; 
    public static final OpticalSensorConfig kOpticSensorConfig = new OpticalSensorConfig(kOpticSensorNAME, K_OPTIC_SENSOR_ID);
}