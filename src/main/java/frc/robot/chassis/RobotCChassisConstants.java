package frc.robot.chassis;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.chassis.ChassisConfig;
import frc.demacia.utils.chassis.Mk5nConstants;
import frc.demacia.utils.chassis.SwerveModuleConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.CancoderConfig;
import frc.demacia.utils.sensors.PigeonConfig;

public class RobotCChassisConstants {

  public static final String NAME = "robot c Chassis";

  public static final int PIGEON_ID = 14;
  public static final Canbus CAN_BUS = Canbus.CANIvore;
  public static final Canbus PIGEON_CAN_BUS = Canbus.CANIvore;
  public static final double STEER_GEAR_RATIO = Mk5nConstants.STEER_GEAR_RATIO;
  public static final double DRIVE_GEAR_RATIO = Mk5nConstants.R2.driveGearRatio;
  public static final double WHEEL_DIAMETER = Mk5nConstants.WHEEL_DIAMETER;
  public static final double METERS_FROM_360_DEGS = Mk5nConstants.R2.metersFrom360Degs;
  public static final double MAX_DRIVE_VELOCITY = 5;
  public static final double RAMP_TIME_STEER = 0.25;

  public static final double STEER_KP = 5;
  public static final double STEER_KI = 0;
  public static final double STEER_KD = 0;
  public static final double STEER_KS = 0.35;
  public static final double STEER_KV = 0.38;
  public static final double STEER_KA = 0.01;

  public static final double DRIVE_KP = 1;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KD = 0;
  public static final double DRIVE_KS = 0.15;
  public static final double DRIVE_KV = 2.4;
  public static final double DRIVE_KA = 0;

  public static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(0.32, 0.27), //FRONT LEFT 
    new Translation2d(0.32, -0.27), //FRONT RIGHT 
    new Translation2d(-0.32, 0.27), //BACK LEFT 
    new Translation2d(-0.32, -0.27), //BACK RIGHT 
  };

  public static final SwerveModuleConfig[] modules = swerveModules(
      new double[] {
        3.124718864923051, //FRONT LEFT
        2.462039164556454, //FRONT RIGHT
        2.9989324403164286, //BACK LEFT
        -2.0156507552817327 //BACK RIGHT
      });

  public static final PigeonConfig PIGEON_CONFIG = new PigeonConfig(NAME + " pigeon", PIGEON_ID, PIGEON_CAN_BUS);

  public static final ChassisConfig CHASSIS_CONFIG = new ChassisConfig(
      NAME,
      modules,
      PIGEON_CONFIG);

  public static final Matrix<N3, N1> STATE_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.05, 0.05, 0 }));

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
              .withBrake(true)
              .withInvert(true)
              .withRadiansMotor(STEER_GEAR_RATIO)
              .withRampTime(RAMP_TIME_STEER),
          new TalonFXConfig(name + " Drive", i * 3 + 1, CAN_BUS)
              .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0, 0, 0)
              .withBrake(false)
              .withMeterMotor(DRIVE_GEAR_RATIO, WHEEL_DIAMETER),
          new CancoderConfig(name + " Cancoder", i * 3 + 3, CAN_BUS)
            .withInvert(true))
          .withPosion(MODULE_LOCATIONS[i])
          .withSteerOffset(offsets[i])
          .withMetersFrom360Degs(METERS_FROM_360_DEGS);
    }
    return ans;
  }
}
