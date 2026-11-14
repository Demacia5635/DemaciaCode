package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.demacia.RobotPose.RobotPose;
import frc.robot.vision.VisionConstants;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotCChassisConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.ClimbSubsystem;

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
  private final ClimbSubsystem climbSubsystem;
  public static DriveCommand driveCommand;
  private final Chassis chassis;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    chassis = Chassis.getInstance();
    climbSubsystem = new ClimbSubsystem();
    RobotPose.initialize(
        RobotCChassisConstants.STATE_STD,
        VisionConstants.visionConfig);

    configureBindings();
    setDefaultCommands();
    setController();
  }

  private void configureBindings() {
    controller.downButton().onTrue(Commands.runOnce(() -> {
      if (climbSubsystem.isCalibrated()) {
        CommandScheduler.getInstance().schedule(new ClimbCommand(climbSubsystem, chassis));    
      }
    }));
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
    return null;
  }
}