package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.SparkFlexConfig;
import frc.demacia.utils.motors.SparkFlexMotor;
import frc.demacia.utils.motors.SparkMaxConfig;
import frc.demacia.utils.motors.SparkMaxMotor;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXConfig;
import frc.demacia.utils.motors.TalonSRXMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.RobotPose.RobotPose;
import frc.robot.vision.VisionConstants;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotCChassisConstants;
import frc.robot.intake.subsystems.Intake;
import frc.robot.intake.IntakeConstants;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.shinua.subsystems.Shinua;
import frc.robot.shinua.ShinuaConstants;
import frc.robot.shinua.commands.ShinuaCommand;
import frc.robot.turret.subsystems.Turret;
import frc.robot.turret.TurretConstants;
import frc.robot.turret.commands.TurretCommand;
import frc.robot.shooter.subsystems.Shooter;
import frc.robot.shooter.ShooterConstants;
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
    // Chassis.initialize(RobotCChassisConstants.CHASSIS_CONFIG);
    // driveCommand = new DriveCommand(Chassis.getInstance(), controller);
    // intake = Intake.getInstance();
    // shinua = Shinua.getInstance();
    // turret = Turret.getInstance();
    // shooter = Shooter.getInstance();

    // RobotPose.initialize(
    //   RobotCChassisConstants.STATE_STD, 
    //   VisionConstants.visionConfig);

    configureBindings();
    setDefaultCommands();
    setController();
  }

  private void configureBindings() {

  }

  private void setDefaultCommands() {
    // Chassis.getInstance().setDefaultCommand(driveCommand);
    // intake.setDefaultCommand(new IntakeCommand());
    // shinua.setDefaultCommand(new ShinuaCommand());
    // turret.setDefaultCommand(new TurretCommand());
    // shooter.setDefaultCommand(new ShooterCommand());
  }

  private void setController() {
    // controller.upButton().onTrue(new InstantCommand(() -> {
    //   intake.setState(IntakeConstants.IntakeStates.SHOOTING);
    //   shinua.setState(ShinuaConstants.ShinuaStates.SHOOTING);
    //   turret.setState(TurretConstants.TurretStates.SHOOTING);
    //   shooter.setState(ShooterConstants.ShooterStates.SHOOTING);
    // }));
    // controller.downButton().onTrue(new InstantCommand(() -> {
    //   intake.setState(IntakeConstants.IntakeStates.INTAKING);
    //   shinua.setState(ShinuaConstants.ShinuaStates.NO_MECANUM);
    //   turret.setState(TurretConstants.TurretStates.SHOOTING);
    //   shooter.setState(ShooterConstants.ShooterStates.TRANCH);
    // }));
    // controller.rightButton().onTrue(new InstantCommand(() -> {
    //   intake.setState(IntakeConstants.IntakeStates.CLOSED);
    //   shinua.setState(ShinuaConstants.ShinuaStates.NOT_SHOOTING);
    //   turret.setState(TurretConstants.TurretStates.SHOOTING);
    //   shooter.setState(ShooterConstants.ShooterStates.TRANCH);
    // }));
    // controller.leftButton().onTrue(new InstantCommand(() -> {
    //   intake.setState(intake.IDLE_STATE);
    //   shinua.setState(shinua.IDLE_STATE);
    //   turret.setState(turret.IDLE_STATE);
    //   shooter.setState(shooter.IDLE_STATE);
    // }));

    
    boolean FLYWHEEL_BRAKE = false;
    boolean FLYWHEEL_INVERT = false;
    double FLYWHEEL_GEAR_RATIO = 1.0;
    double FLYWHEEL_DIAMETER = 2 * 0.0254;
    double RAMP_TIME = 0.6;
    double FLYWHEEL_KP = 0.2;
    double FLYWHEEL_KI = 0.0;
    double FLYWHEEL_KD = 0.0;
    double FLYWHEEL_KS = 0.47705;
    double FLYWHEEL_KV = 0.65384;
    double FLYWHEEL_KA = 0.21786;
    double FLYWHEEL_KG = 0.0;
    double FLYWHEEL_KV2 = 0.0;
    double FLYWHEEL_KCOS = 0.0;

    new TalonFXMotor(new TalonFXConfig("FX", 9, Canbus.Rio)
    .withBrake(FLYWHEEL_BRAKE)
        .withInvert(FLYWHEEL_INVERT)
        .withRampTime(RAMP_TIME)
        .withMeterMotor(FLYWHEEL_GEAR_RATIO, FLYWHEEL_DIAMETER)
        .withPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA, FLYWHEEL_KG, FLYWHEEL_KCOS, FLYWHEEL_KV2));
    new TalonSRXMotor(new TalonSRXConfig("SRX", 10)
    .withBrake(FLYWHEEL_BRAKE)
        .withInvert(FLYWHEEL_INVERT)
        .withRampTime(RAMP_TIME)
        .withMeterMotor(FLYWHEEL_GEAR_RATIO, FLYWHEEL_DIAMETER)
        .withPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA, FLYWHEEL_KG, FLYWHEEL_KCOS, FLYWHEEL_KV2));
    new SparkMaxMotor(new SparkMaxConfig("SparkMax", 18)
    .withBrake(FLYWHEEL_BRAKE)
        .withInvert(FLYWHEEL_INVERT)
        .withRampTime(RAMP_TIME)
        .withMeterMotor(FLYWHEEL_GEAR_RATIO, FLYWHEEL_DIAMETER)
        .withPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA, FLYWHEEL_KG, FLYWHEEL_KCOS, FLYWHEEL_KV2));
    new SparkFlexMotor(new SparkFlexConfig("SparkFlex", 19)
    .withBrake(FLYWHEEL_BRAKE)
        .withInvert(FLYWHEEL_INVERT)
        .withRampTime(RAMP_TIME)
        .withMeterMotor(FLYWHEEL_GEAR_RATIO, FLYWHEEL_DIAMETER)
        .withPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA, FLYWHEEL_KG, FLYWHEEL_KCOS, FLYWHEEL_KV2));
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
    return (new RunCommand(() -> Chassis.getInstance().setSpeedsFieldRel(
      new ChassisSpeeds(1, 0, 0.1)), 
      Chassis.getInstance()).withTimeout(2))
    .andThen(new RunCommand(() -> Chassis.getInstance().setSpeedsFieldRel(
      new ChassisSpeeds(0, 1, 1)), 
      Chassis.getInstance()).withTimeout(2))
    .andThen(new RunCommand(() -> Chassis.getInstance().setSpeedsFieldRel(
      new ChassisSpeeds(-1, 0, -0.5)), 
      Chassis.getInstance()).withTimeout(2))
    .andThen(new RunCommand(() -> Chassis.getInstance().setSpeedsFieldRel(
      new ChassisSpeeds(0, -1, -1)), 
      Chassis.getInstance()).withTimeout(2))
      .andThen(new InstantCommand(() -> Chassis.getInstance().stop(), 
        Chassis.getInstance()));
  }
}