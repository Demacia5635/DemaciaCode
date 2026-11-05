package frc.robot.constants;


import frc.demacia.utils.motors.TalonSRXConfig;
import frc.demacia.utils.sensors.OpticalSensorConfig;
import frc.demacia.utils.sensors.UltraSonicSensorConfig;

public class ConstantsGripper {
    public static final int kSnowblowerID = 32;
    public static final String kSnowBlowerName = "Snow_Blower";
    public static final double DISTANCE_TO_BOTTLE = 0.07;
    public static final double DISTANCE_TO_BOTTLE_CLAMP = 0.1;

    public static final TalonSRXConfig kSnowBlowerConfig = new TalonSRXConfig(
        kSnowblowerID,
        kSnowBlowerName
    );
    public static final int K_OPTIC_SENSOR_ID = 8;
    public static final String kOpticSensorNAME = "Optic_Sensor"; 
    public static final OpticalSensorConfig kOpticSensorConfig = new OpticalSensorConfig(kOpticSensorNAME, K_OPTIC_SENSOR_ID);
      public static final UltraSonicSensorConfig kUltraSonicSensorConfig = new UltraSonicSensorConfig(8, 9, "UltraSonic_Sensor");
} 