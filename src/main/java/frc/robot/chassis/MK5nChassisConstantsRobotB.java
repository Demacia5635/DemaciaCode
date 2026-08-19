package frc.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.chassis.ChassisConfig;
import frc.demacia.utils.chassis.SwerveModuleConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.CancoderConfig;
import frc.demacia.utils.sensors.PigeonConfig;
import frc.demacia.vision.TagPose;



public class MK5nChassisConstantsRobotB {

  
  public static final String NAME = "Robot B Chassis";

  public static final int PIGEON_ID = 14;
  public static final Canbus CAN_BUS = Canbus.CANIvore;
  public static final Canbus PIGEON_CAN_BUS = Canbus.Rio;
  public static final double STEER_GEAR_RATIO = 287.0 / 11.0;
  public static final double DRIVE_GEAR_RATIO = 6.03;
  public static final double WHEEL_DIAMETER = 4 * 0.0254;

  public static final double STEER_KP = 12;
  public static final double STEER_KI = 0;
  public static final double STEER_KD = 0;
  public static final double STEER_KS = 0.08921;
  public static final double STEER_KV = 0.35112;
  public static final double STEER_KA = 0;

  public static final double DRIVE_KP = 6;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KD = 0;
  public static final double DRIVE_KS = 0.03118;
  public static final double DRIVE_KV = 2.16230;
  public static final double DRIVE_KA = 0;

  public static final double MOTION_MAGIC_VEL = 15 * 2 * Math.PI;
  public static final double MOTION_MAGIC_ACCEL = 8 * 2 * Math.PI;
  public static final double MOTION_MAGIC_JERK = 160 * 2 * Math.PI;

  public static final double MAX_DRIVE_VELOCITY = 5d;

  public static final double RAMP_TIME_STEER = 0.25;

  

  public static final Translation2d[] MODULE_LOCATIONS = {
  /* Front Left Posesan: */  new Translation2d(0.32, 0.27),
  /* Front Right Posesan: */  new Translation2d(0.32, -0.27),
  /* Back Left Posesan: */  new Translation2d(-0.32, 0.27),
  /* Back Right Posasen: */   new Translation2d(-0.32, -0.27)
  };

  public static final SwerveModuleConfig[] swerveModules(double[] offsets) {
    SwerveModuleConfig[] ans = new SwerveModuleConfig[4];
    for (int i = 0; i < 4; i++) {
      String name = "Error";
      switch (i) {
        case 0:
          name = "Front Left";
          break;
        case 1:
          name = "Front Right";
          break;
        case 2:
          name = "Back Left";
          break;
        case 3:
          name = "Back Right";
          break;

        default:
          name = "";
          break;
      }

      ans[i] = new SwerveModuleConfig(
          name,
          new TalonFXConfig(i * 3 + 2, CAN_BUS, name + " Steer")
              .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
              .withMotionParam(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
              .withBrake(true)
              .withInvert(true)
              .withRadiansMotor(STEER_GEAR_RATIO)
              .withRampTime(RAMP_TIME_STEER),
          new TalonFXConfig(i * 3 + 1, CAN_BUS, name + " Drive")
              .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
              .withBrake(true)
              .withMeterMotor(DRIVE_GEAR_RATIO, WHEEL_DIAMETER),
          new CancoderConfig(i * 3 + 3, CAN_BUS, name + " Cancoder"))
              .withPosion(MODULE_LOCATIONS[i])
              .withSteerOffset(offsets[i]);
    }
    // for(var a : ans) {
    //   LogManager.log(" module " + a.name + " " + a.position + " " + a.driveConfig.id);
    // }
    return ans;
  }

  public static final SwerveModuleConfig[] modules = swerveModules(
      new double[] {
        /* Front Left Offset: */ -0.256104 * 2 * Math.PI,
        /* Front Right Offset: */  -0.202148 * 2 * Math.PI,
        /* Back Left Offset: */  0.213379 * 2 * Math.PI,
        /* Back Right Offset: */  -0.334961 * 2 * Math.PI
      });

  public static final PigeonConfig PIGEON_CONFIG = new PigeonConfig(PIGEON_ID, PIGEON_CAN_BUS, NAME + "/pigeon");

  public static final ChassisConfig CHASSIS_CONFIG = new ChassisConfig(
      NAME,
      modules,
      PIGEON_CONFIG,
      new TagPose[] {});
}