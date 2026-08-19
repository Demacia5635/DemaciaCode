// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.path.DemaciaTrajectory;
import frc.demacia.path.commands.PathCommand;
import frc.demacia.path.trapzoid.DemaciaTrapezoid;
import frc.demacia.utils.DemaciaUtils;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.LogManager;
// import frc.robot.chassis.RobotBChassisConstants;
// import frc.robot.chassis.chackDriveng;
import frc.robot.chassis.MK5nChassisConstantsRobotB;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable{
  public static boolean isComp = false;
  private static boolean hasRemovedFromLog = false;
  public static boolean isRed = false;
  private List<Translation2d> demaciaPathPoints = new ArrayList<>();
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public CommandController controller = new CommandController(0, ControllerType.kPS5);
  public DemaciaTrapezoid trapezoid = new DemaciaTrapezoid(1, 3);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    new DemaciaUtils(() -> getIsComp(), () -> getIsRed());
    // checkPigen.schedule();
    Chassis.initialize(MK5nChassisConstantsRobotB.CHASSIS_CONFIG);
    DriveCommand driveCommand = new DriveCommand(controller);
    // driveCommand.invertPrecisionMode();
    Chassis.getInstance().setDefaultCommand(driveCommand);
    // Configure the trigger bindings
    configureBindings();
    configurePoints();
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configurePoints() {
    demaciaPathPoints.add(new Translation2d(0, 0));
    demaciaPathPoints.add(new Translation2d(2, 0));
    demaciaPathPoints.add(new Translation2d(2, 2));
    demaciaPathPoints.add(new Translation2d(1, 1));
    demaciaPathPoints.add(new Translation2d(0, 0));
  }

  private void configureBindings() {
    
  }

  public static boolean getIsRed() {
    return isRed;
  }

  public static void setIsRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  public static boolean getIsComp() {
    return isComp;
  }

  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if(!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("isRed", RobotContainer::getIsRed, RobotContainer::setIsRed);
    builder.addBooleanProperty("isComp", RobotContainer::getIsComp, RobotContainer::setIsComp);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathCommand(new DemaciaTrajectory(demaciaPathPoints));
    // return new chackDriveng();
  }
}