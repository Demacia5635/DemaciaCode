package frc.robot.constants;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

public class ConstantsGripper {
    public static final int kSnowblowerID = 1;
    public static final boolean kIsBrake = true;
    public static final Canbus kSnowBlowerCanbus = Canbus.Rio;
    public static final String kSnowBlowerName = "Snow_Blower";
    
    public static final TalonFXConfig kSnowBlowerConfig = new TalonFXConfig(kSnowblowerID, kSnowBlowerCanbus, kSnowBlowerName)
        .withBrake(kIsBrake);
}