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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.demacia.RobotPose.Vision.VisionConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.QuestConfig;

public class RobotDChassisConstants {
  public static final String NAME = "robot d Chassis";

  public static final int PIGEON_ID = 7;
  public static final Canbus CAN_BUS = Canbus.CANIvore;
  public static final Canbus PIGEON_CAN_BUS = Canbus.CANIvore;
  public static final double STEER_GEAR_RATIO = Mk5nConstants.STEER_GEAR_RATIO;
  public static final double DRIVE_GEAR_RATIO = Mk5nConstants.R1.driveGearRatio;
  public static final double WHEEL_DIAMETER = Mk5nConstants.WHEEL_DIAMETER;
  public static final double METERS_FROM_360_DEGS = Mk5nConstants.R1.metersFrom360Degs;
  public static final double MAX_DRIVE_VELOCITY = 5;
  public static final double RAMP_TIME_STEER = 0.25;

  public static final double STEER_KP = 0;
  public static final double STEER_KI = 0;
  public static final double STEER_KD = 0;
  public static final double STEER_KS = 0;
  public static final double STEER_KV = 0;
  public static final double STEER_KA = 0;

  public static final double DRIVE_KP = 0;
  public static final double DRIVE_KI = 0;
  public static final double DRIVE_KD = 0;
  public static final double DRIVE_KS = 0;
  public static final double DRIVE_KV = 0;
  public static final double DRIVE_KA = 0;

  public static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(0, 0), //FRONT LEFT // TODO
    new Translation2d(0, 0), //FRONT RIGHT // TODO
    new Translation2d(0, 0), //BACK LEFT // TODO
    new Translation2d(0, 0), //BACK RIGHT // TODO
  };

  public static final SwerveModuleConfig[] modules = swerveModules(
      new double[] {
        0, //FRONT LEFT // TODO
        0, //FRONT RIGHT // TODO
        0, //BACK LEFT // TODO
        0 //BACK RIGHT // TODO
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
              .withInvert(false) //TODO
              .withCurrent(40)
              .withRadiansMotor(STEER_GEAR_RATIO)
              .withRampTime(RAMP_TIME_STEER),
          new TalonFXConfig(name + " Drive", i * 3 + 1, CAN_BUS)
              .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0, 0, 0)
              .withBrake(true)
              .withInvert(false) //TODO
              .withCurrent(40)
              .withMeterMotor(DRIVE_GEAR_RATIO, WHEEL_DIAMETER),
          new CancoderConfig(name + " Cancoder", i * 3 + 3, CAN_BUS)
            .withInvert(false)) //TODO
          .withPosion(MODULE_LOCATIONS[i])
          .withSteerOffset(offsets[i])
          .withMetersFrom360Degs(METERS_FROM_360_DEGS);
    }
    return ans;
  }
  public class VisionConstants {
      public static final String L_NAME = "l";
      public static final Transform3d L_OFFSET = new Transform3d(
          new Translation3d(0.0, 0.0, 0.0), // TODO
          new Rotation3d(0.0, 0.0, 0.0)); // TODO
      public static final Matrix<N3, N1> L_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.03, 0.03, 0.0 })); // TODO
      public static final LimelightTagCamera3dConfig L_CONFIG = new LimelightTagCamera3dConfig(L_NAME, L_OFFSET, L_STD);

      public static final String Q_NAME = "q";
      public static final Transform3d Q_OFFSET = new Transform3d(
          new Translation3d(0.0, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, 0.0));
      public static final Matrix<N3, N1> Q_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.03, 0.03, 0.0 }));
      public static final QuestConfig Q_CONFIG = new QuestConfig(Q_NAME, Q_OFFSET, Q_STD);

      public static final VisionConfig visionConfig = new VisionConfig()
          .addSource(L_CONFIG)
          .addSource(Q_CONFIG)
          ;
  }
}
