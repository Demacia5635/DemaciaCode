// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.DemaciaUtils;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.LogManager;
import frc.robot.commands.EjectCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;


public class RobotContainer implements Sendable{

  public static boolean isComp = false;
  private static boolean hasRemovedFromLog = false;
  public static boolean isRed = false;


  // Subsystem & controller:
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final CommandController controller = new CommandController(Constants.ControllerConstants.kPS5ControllerPort, ControllerType.kPS5); // Make sure the port and the controller type are corect.



  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    new DemaciaUtils(() -> getIsComp(), () -> getIsRed());
    
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {

    controller.downButton().whileTrue(new IntakeCommand(intake));
    controller.upButton().whileTrue(new EjectCommand(intake));

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


  public Command getAutonomousCommand() {
   return null;
  }
}