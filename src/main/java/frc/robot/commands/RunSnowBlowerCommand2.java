package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.log.LogManager;
import frc.robot.subsystems.SubsystemSnowBlower;

public class RunSnowBlowerCommand2 extends Command {
  final double duty2;
  final SubsystemSnowBlower snowBlower2;
  /** Creates a new RunSnowBlowerCommand. */
    public RunSnowBlowerCommand2(SubsystemSnowBlower subsystem, double duty2) {
        this.snowBlower2 = subsystem;
        this.duty2 = duty2;
        addRequirements(this.snowBlower2);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    snowBlower2.setPower(duty2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    snowBlower2.setPower(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
