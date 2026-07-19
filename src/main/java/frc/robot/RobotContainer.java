package frc.robot;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;
import edu.wpi.first.wpilibj2.command.Command;
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
  public TalonFXSimState sim2;
  public TalonFXSimState sim3;
  public TalonFXSimState sim4;
  public TalonFXSimState sim5;
  public TalonFXSimState sim6;
  public TalonFXSimState sim7;
  public TalonFXSimState sim8;
  public TalonFXSimState sim9;
  public TalonFXSimState sim10;
  public TalonFXSimState sim11;
  public TalonFXSimState sim12;
  public TalonFXSimState sim13;
  public TalonFXSimState sim14;
  public TalonFXSimState sim15;
  public TalonFXSimState sim16;
  public TalonFXSimState sim17;
  public TalonFXSimState sim18;
  public TalonFXSimState sim19;
  public TalonFXSimState sim20;
  public TalonFXSimState sim21;
  public TalonFXSimState sim22;
  public TalonFXSimState sim23;
  public TalonFXSimState sim24;
  public TalonFXSimState sim25;
  public TalonFXSimState sim26;
  public TalonFXSimState sim27;
  public TalonFXSimState sim28;
  public TalonFXSimState sim29;
  public TalonFXSimState sim30;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    Chassis.initialize(RobotChassisConstants.CHASSIS_CONFIG);
    // driveCommand = new DriveCommand(Chassis.getInstance(), controller);
    // shootet = Shootet.getInstance();
    sim = (new TalonFXMotor(new TalonFXConfig(1, HoodConstants.HOOD_CANBUS, "1"))).getSimState();
    sim2 = (new TalonFXMotor(new TalonFXConfig(2, HoodConstants.HOOD_CANBUS, "2"))).getSimState();
    sim3 = (new TalonFXMotor(new TalonFXConfig(3, HoodConstants.HOOD_CANBUS, "3"))).getSimState();
    sim4 = (new TalonFXMotor(new TalonFXConfig(4, HoodConstants.HOOD_CANBUS, "4"))).getSimState();
    sim5 = (new TalonFXMotor(new TalonFXConfig(5, HoodConstants.HOOD_CANBUS, "5"))).getSimState();
    sim6 = (new TalonFXMotor(new TalonFXConfig(6, HoodConstants.HOOD_CANBUS, "6"))).getSimState();
    sim7 = (new TalonFXMotor(new TalonFXConfig(7, HoodConstants.HOOD_CANBUS, "7"))).getSimState();
    sim8 = (new TalonFXMotor(new TalonFXConfig(8, HoodConstants.HOOD_CANBUS, "8"))).getSimState();
    sim9 = (new TalonFXMotor(new TalonFXConfig(9, HoodConstants.HOOD_CANBUS, "9"))).getSimState();
    sim10 = (new TalonFXMotor(new TalonFXConfig(10, HoodConstants.HOOD_CANBUS, "10"))).getSimState();
    sim11 = (new TalonFXMotor(new TalonFXConfig(11, HoodConstants.HOOD_CANBUS, "11"))).getSimState();
    sim12 = (new TalonFXMotor(new TalonFXConfig(12, HoodConstants.HOOD_CANBUS, "12"))).getSimState();
    sim13 = (new TalonFXMotor(new TalonFXConfig(13, HoodConstants.HOOD_CANBUS, "13"))).getSimState();
    sim14 = (new TalonFXMotor(new TalonFXConfig(14, HoodConstants.HOOD_CANBUS, "14"))).getSimState();
    sim15 = (new TalonFXMotor(new TalonFXConfig(15, HoodConstants.HOOD_CANBUS, "15"))).getSimState();
    sim16 = (new TalonFXMotor(new TalonFXConfig(16, HoodConstants.HOOD_CANBUS, "16"))).getSimState();
    sim17 = (new TalonFXMotor(new TalonFXConfig(17, HoodConstants.HOOD_CANBUS, "17"))).getSimState();
    sim18 = (new TalonFXMotor(new TalonFXConfig(18, HoodConstants.HOOD_CANBUS, "18"))).getSimState();
    sim19 = (new TalonFXMotor(new TalonFXConfig(19, HoodConstants.HOOD_CANBUS, "19"))).getSimState();
    sim20 = (new TalonFXMotor(new TalonFXConfig(20, HoodConstants.HOOD_CANBUS, "20"))).getSimState();
    sim21 = (new TalonFXMotor(new TalonFXConfig(21, HoodConstants.HOOD_CANBUS, "21"))).getSimState();
    sim22 = (new TalonFXMotor(new TalonFXConfig(22, HoodConstants.HOOD_CANBUS, "22"))).getSimState();
    sim23 = (new TalonFXMotor(new TalonFXConfig(23, HoodConstants.HOOD_CANBUS, "23"))).getSimState();
    sim24 = (new TalonFXMotor(new TalonFXConfig(24, HoodConstants.HOOD_CANBUS, "24"))).getSimState();
    sim25 = (new TalonFXMotor(new TalonFXConfig(25, HoodConstants.HOOD_CANBUS, "25"))).getSimState();
    sim26 = (new TalonFXMotor(new TalonFXConfig(26, HoodConstants.HOOD_CANBUS, "26"))).getSimState();
    sim27 = (new TalonFXMotor(new TalonFXConfig(27, HoodConstants.HOOD_CANBUS, "27"))).getSimState();
    sim28 = (new TalonFXMotor(new TalonFXConfig(28, HoodConstants.HOOD_CANBUS, "28"))).getSimState();
    sim29 = (new TalonFXMotor(new TalonFXConfig(29, HoodConstants.HOOD_CANBUS, "29"))).getSimState();
    sim30 = (new TalonFXMotor(new TalonFXConfig(30, HoodConstants.HOOD_CANBUS, "30"))).getSimState();

    configureBindings();
    setDefaultCommands();
    setController();
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