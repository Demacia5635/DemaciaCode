package frc.robot;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotChassisConstants;
import frc.robot.intake.subsystems.Intake;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.shinua.subsystems.Shinua;
import frc.robot.shinua.commands.ShinuaCommand;
import frc.robot.turret.subsystems.Turret;
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    Chassis.initialize(RobotChassisConstants.CHASSIS_CONFIG);
    driveCommand = new DriveCommand(Chassis.getInstance(), controller);
    intake = Intake.getInstance();
    shinua = Shinua.getInstance();
    turret = Turret.getInstance();
    shooter = Shooter.getInstance();

    configureBindings();
    setDefaultCommands();
    setController();
  }

  private void configureBindings() {

  }

  private void setDefaultCommands() {
    Chassis.getInstance().setDefaultCommand(driveCommand);
    intake.setDefaultCommand(new IntakeCommand());
    shinua.setDefaultCommand(new ShinuaCommand());
    turret.setDefaultCommand(new TurretCommand());
    shooter.setDefaultCommand(new ShooterCommand());
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