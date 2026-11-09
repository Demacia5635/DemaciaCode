package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemKraken;

public class Krakenforwadbackcommand extends Command {
  final double speed;
  final SubsystemKraken Kraken1;
  final SubsystemKraken Kraken2;
  /** Creates a new RunSnowBlowerCommand. */
    public Krakenforwadbackcommand(SubsystemKraken subsystem, double speed) {
        this.Kraken1 = subsystem;
        this.Kraken2 = subsystem;
        this.speed = speed;
        addRequirements(this.Kraken1, this.Kraken2);

    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Kraken1.setPowerforwadandback(speed);
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
