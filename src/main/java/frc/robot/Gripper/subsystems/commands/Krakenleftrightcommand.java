package frc.robot.Gripper.subsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.demacia.utils.controller.CommandController;
import frc.robot.Gripper.subsystems.Subsystem.SubsystemKraken;

public class Krakenleftrightcommand extends Command {
  private double speed;
  private SubsystemKraken Kraken2;
  public CommandController controller;
    public Krakenleftrightcommand(SubsystemKraken subsystem, double speed) {
        this.Kraken2 = subsystem;
        this.speed = speed;
        addRequirements(Kraken2);
        this.controller = controller;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Kraken2.rollmove(speed);
    Kraken2.Is_Out_Max();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
