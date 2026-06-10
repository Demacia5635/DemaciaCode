// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.path.DemaciaTrajectoryGood;
import frc.demacia.path.PathCommand;
import frc.demacia.utils.DemaciaUtils;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.LogManager;
import frc.robot.chassis.RobotBChassisConstants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable{
  private Chassis chassis;
  public static boolean isComp = false;
  private static boolean hasRemovedFromLog = false;
  public static boolean isRed = false;
  public CommandController controller = new CommandController(0, ControllerType.kPS5);

  private List<Pose2d> demaciaPathPoints = new ArrayList<>();

  // The robot's subsystems and commands are defined here...


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    new DemaciaUtils(() -> getIsComp(), () -> getIsRed());
    chassis = new Chassis(RobotBChassisConstants.CHASSIS_CONFIG);
    chassis.setDefaultCommand(new DriveCommand(chassis, controller));
    // Configure the trigger bindings
    configureBindings();
    configPoint();
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

  private void configPoint() {
    demaciaPathPoints.add(new Pose2d(0, 0, new Rotation2d()));
    demaciaPathPoints.add(new Pose2d(2, 2, new Rotation2d()));
    demaciaPathPoints.add(new Pose2d(4, 0, new Rotation2d()));
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
    return new PathCommand(new DemaciaTrajectoryGood(demaciaPathPoints), chassis);
  }
}