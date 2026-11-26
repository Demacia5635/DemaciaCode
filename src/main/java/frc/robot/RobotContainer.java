package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotCChassisConstants;
import frc.robot.vision.VisionConstants;

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

  private boolean isComp = false;
  private static boolean hasRemovedFromLog = false;
  private boolean isRed = false;
  // The robot's subsystems and commands are defined here...
  private Chassis chassis;
  private DriveCommand driveCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandController controller = new CommandController(0, ControllerType.kPS5);

  /**
   * The container for the root. Contains subsystems, OI devices, and commands.
   */

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * 
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    Chassis.initialize(RobotCChassisConstants.CHASSIS_CONFIG);
    driveCommand = new DriveCommand(Chassis.getInstance(), controller);

    RobotPose.initialize(
        RobotCChassisConstants.STATE_STD,
        VisionConstants.visionConfig);

    setDefaultCommands();
    setController();
  }

  private void setDefaultCommands() {
    Chassis.getInstance().setDefaultCommand(driveCommand);
  }

  private void setController() {
    controller.rightButton().onTrue(new InstantCommand(() -> driveCommand.invertPrecisionMode()));
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