package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemSnowBlower;

public class RunSnowBlowerCommand extends Command {
  final double speed;
  final SubsystemSnowBlower snowBlower;
  /** Creates a new RunSnowBlowerCommand. */
    public RunSnowBlowerCommand(SubsystemSnowBlower subsystem, double speed) {
        this.snowBlower = subsystem;
        this.speed = 1;
        addRequirements(this.snowBlower);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    snowBlower.setPower(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    snowBlower.setPower(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
