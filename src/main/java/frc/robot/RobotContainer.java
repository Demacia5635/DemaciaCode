package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.Path;
import frc.demacia.path.commands.followPathCommand;
import frc.robot.vision.VisionConstants;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotCChassisConstants;

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

  public static DriveCommand driveCommand;

  private Path path;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    Chassis.initialize(RobotCChassisConstants.CHASSIS_CONFIG);
    driveCommand = new DriveCommand(Chassis.getInstance(), controller);

    RobotPose.initialize(
      RobotCChassisConstants.STATE_STD, 
      VisionConstants.visionConfig);

    path = new Path(new Pose2d[] {
      new Pose2d(0,0, new Rotation2d()), 
      new Pose2d(0, 5, new Rotation2d()), 
      new Pose2d(2, 0.5, new Rotation2d()), 
      new Pose2d(4.5, 0.25, new Rotation2d()), 
      new Pose2d(6, 2, new Rotation2d())
    });
    // path = new Path(new Pose2d[] {
    //   new Pose2d(0,3, new Rotation2d()), 
    //   new Pose2d(10, 5, new Rotation2d())
    // });

    configureBindings();
    setDefaultCommands();
    setController();
  }

  private void configureBindings() {

  }

  private void setDefaultCommands() {
    Chassis.getInstance().setDefaultCommand(driveCommand);
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
    // return new RunCommand(() -> Chassis.getInstance().setSpeedsFieldRel(new ChassisSpeeds(1, 0, 1)), Chassis.getInstance());
    return new followPathCommand(path);
  }
}