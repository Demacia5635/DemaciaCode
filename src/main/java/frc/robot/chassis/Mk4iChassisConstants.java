package frc.robot.chassis;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.chassis.ChassisConfig;
import frc.demacia.utils.chassis.SwerveModuleConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.CancoderConfig;
import frc.demacia.utils.sensors.PigeonConfig;

public class Mk4iChassisConstants {
  public static final String NAME = "Mk4i Chassis";

  public static final int PIGEON_ID = 14;
  public static final Canbus CAN_BUS = Canbus.Rio;
  public static final Canbus PIGEON_CAN_BUS = Canbus.Rio;
  public static final double STEER_GEAR_RATIO = 150.0 / 7.0; 
  public static final double DRIVE_GEAR_RATIO = 8.14; 
  public static final double WHEEL_DIAMETER = 4 * 0.0254; 

  public static final double STEER_KP = 1;  // TODO
  public static final double STEER_KI = 0;  // TODO
  public static final double STEER_KD = 0;  // TODO
  public static final double STEER_KS = 0;  // TODO
  public static final double STEER_KV = 0;  // TODO
  public static final double STEER_KA = 0;  // TODO

  public static final double DRIVE_KP = 1;  // TODO
  public static final double DRIVE_KI = 0;  // TODO
  public static final double DRIVE_KD = 0;  // TODO
  public static final double DRIVE_KS = 0;  // TODO
  public static final double DRIVE_KV = 0;  // TODO
  public static final double DRIVE_KA = 0;  // TODO

  public static final double MAX_DRIVE_VELOCITY = 5; 
  public static final double RAMP_TIME_STEER = 0.25; 

  public static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(0.4, 0.36), //FRONT LEFT 
    new Translation2d(0.4, -0.36), //FRONT RIGHT 
    new Translation2d(-0.4, 0.36), //BACK LEFT 
    new Translation2d(-0.4, -0.36), //BACK RIGHT 
  };

  public static final SwerveModuleConfig[] modules = swerveModules(
      new double[] {
        -1.8361750030991124, //FRONT LEFT
        -0.7547185476397354, //FRONT RIGHT
        -1.4894953450369577, //BACK LEFT
        -2.6476508398906167 //BACK RIGHT
      });

  public static final PigeonConfig PIGEON_CONFIG = new PigeonConfig(NAME + " pigeon", PIGEON_ID, PIGEON_CAN_BUS);

  public static final ChassisConfig CHASSIS_CONFIG = new ChassisConfig(
      NAME,
      modules,
      PIGEON_CONFIG);

  public static final Matrix<N3, N1> STATE_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.3, 0.3, 0 })); 

  public static final double METERS_FROM_360_DEGS = 0.2;

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
              .withInvert(false)
              .withRadiansMotor(STEER_GEAR_RATIO)
              .withRampTime(RAMP_TIME_STEER),
          new TalonFXConfig(name + " Drive", i * 3 + 1, CAN_BUS)
              .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0, 0, 0)
              .withBrake(true)
              .withMeterMotor(DRIVE_GEAR_RATIO, WHEEL_DIAMETER),
          new CancoderConfig(name + " Cancoder", i * 3 + 3, CAN_BUS))
          .withPosion(MODULE_LOCATIONS[i])
          .withSteerOffset(offsets[i])
          .withMetersFrom360Degs(METERS_FROM_360_DEGS);
    }
    return ans;
  }
}
