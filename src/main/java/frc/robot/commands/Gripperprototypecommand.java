// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.sensors.OpticalSensor;
import frc.robot.subsystems.GreeperPro;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Gripperprototypecommand extends Command {
  /** Creates a new Gripperprototypecommand. */
  public CommandController controller;
  public GreeperPro grepper_sub;
  public Timer timer;
  public Gripperprototypecommand(CommandController controller, GreeperPro greeperPro) {
    this.controller = controller;
    this.grepper_sub = greeperPro;
    timer = new Timer();
    addRequirements(greeperPro);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(grepper_sub.getCurrent() > 25){
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
    if (grepper_sub.getOpticSensor() == true && timer()< 0.1)
      

    
    if (grepper_sub.getOpticSensor() <= 0.5 ){
      grepper_sub.setPower(0.5);
    } else {
      grepper_sub.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
