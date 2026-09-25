package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.RobotPose.Estimation.DemaciaPoseEstimator.OdometryData;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer.Range;
import frc.robot.chassis.RobotChassisConstants;
import frc.robot.intake.subsystems.Intake;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.shinua.subsystems.Shinua;
import frc.robot.shinua.commands.ShinuaCommand;
import frc.robot.turret.subsystems.Turret;
import frc.robot.vision.VisionConstants;
import frc.robot.turret.commands.TurretCommand;
import frc.robot.shooter.subsystems.Shooter;
import frc.robot.shooter.commands.ShooterCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
* the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable {

  public static CommandController controller = new CommandController(0, ControllerType.kPS5);

  private Intake intake;
  private Shinua shinua;
  private Turret turret;
  private Shooter shooter;
  public static DriveCommand driveCommand;
  /** Used for collision detection in odometry, see {@link #getAccelerationFromRoboRio()}. */
  private final BuiltInAccelerometer roboRioAccelerometer = new BuiltInAccelerometer(Range.k8G);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    Chassis.initialize(RobotChassisConstants.CHASSIS_CONFIG);
    // driveCommand = new DriveCommand(Chassis.getInstance(), controller);
    // intake = Intake.getInstance();
    // shinua = Shinua.getInstance();
    // turret = Turret.getInstance();
    // shooter = Shooter.getInstance();

    RobotPose.initialize(
      ()->new OdometryData(Chassis.getInstance().getGyroAngle(), Chassis.getInstance().getModulePositions(), getAccelerationFromRoboRio()),
      Chassis.getInstance().getModulePositions(), 
      Chassis.getInstance().getModuleLocations(), 
      RobotChassisConstants.stateStd, 
      VisionConstants.visionConfig);

    configureBindings();
    setDefaultCommands();
    setController();
  }

  /**
   * Horizontal acceleration of the robot (robot relative, m/s^2) for collision detection, from
   * the roboRIO's built-in accelerometer.
   *
   * <p>Why the roboRIO and not the Pigeon: the roboRIO reads up to 8 g, the Pigeon's acceleration
   * signal stops at 2 g, so every hit read the same ~2 g and the threshold had to sit close to
   * normal driving. The roboRIO is also read right now in the loop (the Pigeon's value comes over
   * CAN, up to 10 ms old) and can't disconnect and leave a stale value. Its downside is that it's
   * usually not at the robot center, so the w^2 * r it reads while spinning is removed here (see
   * {@link RobotChassisConstants#ROBORIO_POSITION}).
   *
   * <p>In simulation it reads 0, so no collisions are detected there.
   */
  private Translation2d getAccelerationFromRoboRio() {
    Translation2d measured = new Translation2d(roboRioAccelerometer.getX(), roboRioAccelerometer.getY())
        .times(9.81)
        .rotateBy(RobotChassisConstants.ROBORIO_YAW);
    // Spinning at w pulls a point at r toward the center with w^2 * r. That isn't a hit.
    double yawRate = Chassis.getInstance().getGyroAngularVelocity();
    return measured.plus(RobotChassisConstants.ROBORIO_POSITION.times(yawRate * yawRate));
  }

  private void configureBindings() {

  }

  private void setDefaultCommands() {
    Chassis.getInstance().setDefaultCommand(driveCommand);
    // intake.setDefaultCommand(new IntakeCommand());
    // shinua.setDefaultCommand(new ShinuaCommand());
    // turret.setDefaultCommand(new TurretCommand());
    // shooter.setDefaultCommand(new ShooterCommand());
  }

  private void setController() {

  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}