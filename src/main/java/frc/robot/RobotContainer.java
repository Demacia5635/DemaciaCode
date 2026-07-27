package frc.robot;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXConfig;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.SparkMaxConfig;
import frc.demacia.utils.motors.SparkMaxMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotChassisConstants;

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    Chassis.initialize(RobotChassisConstants.CHASSIS_CONFIG);
    driveCommand = new DriveCommand(Chassis.getInstance(), controller);

    configureBindings();
    setDefaultCommands();
    setController();

    new SparkMaxMotor(
      new SparkMaxConfig(20, "test motor")
        .withPID(0, 0, 0, 0, 0, 0, 0, 0, 0)
        .withInvert(false)
        .withRadiansMotor(2)
    );

    new TalonFXMotor(
      new TalonFXConfig(20, Canbus.Rio, "talon test motor")
        .withPID(0, 0, 0, 0, 0, 0, 0, 0, 0)
        .withInvert(false)
        .withRadiansMotor(2)
    );

    new TalonSRXMotor(
      new TalonSRXConfig(20, "talonSRX test motor")
        .withPID(0, 0, 0, 0, 0, 0, 0, 0, 0)
        .withInvert(false)
        .withRadiansMotor(2)
    );

  }

  private void configureBindings() {

  }

  private void setDefaultCommands() {
    // Chassis.getInstance().setDefaultCommand(driveCommand);
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