package frc.robot.constants;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

public class ConstantsGripper {
    public static final int kKrakenID = -1;
    public static final boolean kIsBrake = true;
    public static final Canbus kSKrakencanbus = Canbus.Rio;
    public static final String kKrakename = "Snow_Blower";
    
    public static final TalonFXConfig kSnowBlowerConfig = new TalonFXConfig(kKrakenID, kSKrakencanbus, kKrakename)
        .withBrake(kIsBrake);


            public static final int kKrakenID2 = -1;
    public static final boolean kIsBrake2 = true;
    public static final Canbus kSKrakencanbus2 = Canbus.Rio;
    public static final String kKrakename2 = "Snow_Blower"; 
    
    public static final TalonFXConfig kSnowBlowerConfig2 = new TalonFXConfig(kKrakenID2, kSKrakencanbus2, kKrakename2)
        .withBrake(kIsBrake);


    public static final double Kp = -1;
    public static final double Ki =-1;
    public static final double Kd = -1;
    public static final double Ks = -1; //TODO: make all the מה שצריך



}
