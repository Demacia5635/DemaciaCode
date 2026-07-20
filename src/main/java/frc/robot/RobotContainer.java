package frc.robot;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Data;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotChassisConstants;
import frc.robot.shootet.ShootetConstants;
import frc.robot.shootet.ShootetConstants.HoodConstants;
import frc.robot.shootet.subsystems.Shootet;

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

  private Shootet shootet;
  public static DriveCommand driveCommand;

  public TalonFXSimState sim;
  public TalonFXMotor motor;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Chassis.initialize(RobotChassisConstants.CHASSIS_CONFIG);
    // driveCommand = new DriveCommand(Chassis.getInstance(), controller);
    // shootet = Shootet.getInstance();
    motor = new TalonFXMotor(new TalonFXConfig(30, HoodConstants.HOOD_CANBUS, "front t"));
    sim = (motor).getSimState();

    configureBindings();
    setDefaultCommands();
    setController();
    
    SmartDashboard.putData("RC", this);
    Log.log("initialize RC");
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