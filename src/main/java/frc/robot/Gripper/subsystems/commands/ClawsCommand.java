// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.subsystems.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.sensors.OpticalSensor;
import frc.robot.Gripper.subsystems.constants.ConstantsGripper;
import frc.robot.Gripper.subsystems.subsystems.SubsystemGripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class  ClawsCommand extends Command {
  /** Creates a new Gripperprototypecommand. */
  public CommandController controller;
  public SubsystemGripper gripper_sub;
  public Timer timer;
  public boolean isCurrentReady;
  public ClawsCommand(SubsystemGripper griperPro) {
    this.gripper_sub = griperPro;
    timer = new Timer();
    addRequirements(griperPro);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      isCurrentReady = gripper_sub.getCurrent() > 3.5;
      if(ConstantsGripper.DISTANCE_TO_BOTTLE < gripper_sub.getUltraSonicSensor() && ConstantsGripper.DISTANCE_TO_BOTTLE_CLAMP > gripper_sub.getUltraSonicSensor() && !isCurrentReady){
        gripper_sub.setPower(0);
        LogManager.log("stop");
      }
      else if(ConstantsGripper.DISTANCE_TO_BOTTLE > gripper_sub.getUltraSonicSensor() && ConstantsGripper.DISTANCE_TO_BOTTLE_CLAMP > gripper_sub.getUltraSonicSensor() && !isCurrentReady ){
        gripper_sub.setPower(0.6);
          LogManager.log("move");
      }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  gripper_sub.setPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isCurrentReady;

  }
}