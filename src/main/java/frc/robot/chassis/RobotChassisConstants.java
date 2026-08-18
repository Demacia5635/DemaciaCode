package frc.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.chassis.ChassisConfig;
import frc.demacia.utils.chassis.SwerveModuleConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.CancoderConfig;
import frc.demacia.utils.sensors.PigeonConfig;

public class RobotChassisConstants {

  public static final String NAME = "robot Chassis";

  public static final int PIGEON_ID = 14; 
  public static final Canbus CAN_BUS = Canbus.CANIvore; 
  public static final Canbus PIGEON_CAN_BUS = Canbus.Rio; 
  public static final double STEER_GEAR_RATIO = 287d / 11d; 
  public static final double DRIVE_GEAR_RATIO = 6.03; 
  public static final double WHEEL_DIAMETER = 0.1016; 

  public static final double STEER_KP = 3; 
  public static final double STEER_KI = 0.0; 
  public static final double STEER_KD = 0.0; 
  public static final double STEER_KS = 0.0; 
  public static final double STEER_KV = 0.0; 
  public static final double STEER_KA = 0.0; 

  public static final double DRIVE_KP = 3; 
  public static final double DRIVE_KI = 0.0; 
  public static final double DRIVE_KD = 0.0; 
  public static final double DRIVE_KS = 0.0; 
  public static final double DRIVE_KV = 0.0; 
  public static final double DRIVE_KA = 0.0; 

  public static final double STEER_MOTION_MAGIC_VEL = 100.0; 
  public static final double STEER_MOTION_MAGIC_ACCEL = 50.0; 
  public static final double STEER_MOTION_MAGIC_JERK = 1000.0; 

  public static final double MAX_DRIVE_VELOCITY = 5.0; 
  public static final double RAMP_TIME_STEER = 0.25; 

  public static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(0.295, 0.395), //FRONT LEFT  // TODO
    new Translation2d(0.295, -0.395), //FRONT RIGHT  // TODO
    new Translation2d(-0.295, 0.395), //BACK LEFT  // TODO
    new Translation2d(-0.295, -0.395), //BACK RIGHT  // TODO
  };

  public static final SwerveModuleConfig[] modules = swerveModules(
      new double[] {
        0, //FRONT LEFT  // TODO
        0, //FRONT RIGHT  // TODO
        0, //BACK LEFT  // TODO
        0 //BACK RIGHT  // TODO
      });

  public static final PigeonConfig PIGEON_CONFIG = new PigeonConfig(NAME + " pigeon", PIGEON_ID, PIGEON_CAN_BUS);

  public static final ChassisConfig CHASSIS_CONFIG = new ChassisConfig(
      NAME,
      modules,
      PIGEON_CONFIG);

  public static final SwerveModuleConfig[] swerveModules(double[] offsets) {
    SwerveModuleConfig[] ans = new SwerveModuleConfig[4];
    for (int i = 0; i < 4; i++) {
      String name = "Error";
      switch (i) {
        case 0: name = "Front Left"; break;
        case 1: name = "Front Right"; break;
        case 2: name = "Back Left"; break;
        case 3: name = "Back Right"; break;
      }

      ans[i] = new SwerveModuleConfig(
          name,
          new TalonFXConfig(name + " Steer", i * 3 + 2, CAN_BUS)
              .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0, 0, 0)
              .withMotionParam(STEER_MOTION_MAGIC_VEL, STEER_MOTION_MAGIC_ACCEL, STEER_MOTION_MAGIC_JERK)
              .withBrake(true)
              .withInvert(false)
              .withRadiansMotor(STEER_GEAR_RATIO)
              .withRampTime(RAMP_TIME_STEER),
          new TalonFXConfig(name + " Drive", i * 3 + 1, CAN_BUS)
              .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0, 0, 0)
              .withBrake(true)
              .withInvert(true)
              .withMeterMotor(DRIVE_GEAR_RATIO, WHEEL_DIAMETER),
          new CancoderConfig(name + " Cancoder", i * 3 + 3, CAN_BUS))
          .withPosion(MODULE_LOCATIONS[i])
          .withSteerOffset(offsets[i]);
    }
    return ans;
  }
}
