package frc.robot;

import java.security.PublicKey;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.vision.utils.Vision;
import frc.demacia.wpilibSwerve.command.SwerveCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.DemaciaTrajectory;
import frc.demacia.path.commands.PathCommand;
import frc.robot.vision.VisionConstants;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.MK5nChassisConstansRobotC;
import frc.robot.chassis.RobotCChassisConstants;
import edu.wpi.first.math.geometry.Translation2d;

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

  public static SwerveCommand driveCommand;
  public static PathCommand pathCommand;
  public List<Translation2d> points = new ArrayList<Translation2d>();
  public DemaciaTrajectory trajectory;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    Chassis.initialize(MK5nChassisConstansRobotC.CHASSIS_CONFIG);
    driveCommand = new SwerveCommand(controller, new Vision((VisionConstants.Tags.TAGS_ARRAY)));
    addPoint();

    RobotPose.initialize(
      RobotCChassisConstants.STATE_STD, 
      VisionConstants.visionConfig);

    configureBindings();
    setDefaultCommands();
    setController();
  }

  private void configureBindings() {

  }

  private void setDefaultCommands() {
    Chassis.getInstance().setDefaultCommand(driveCommand);
    controller.leftButton().onTrue(new PathCommand(new DemaciaTrajectory(points), driveCommand));
  }

  private void setController() {

  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

  public void addPoint(){
    points.add(new Translation2d(0, 0));
    points.add(new Translation2d(1, 1));
    points.add(new Translation2d(2, 1));
    points.add(new Translation2d(3, 0));
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