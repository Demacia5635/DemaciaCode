// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.path.trapzoid.DemaciaTrapezoid;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class chackDriveng extends Command {
  /** Creates a new chackDriveng. */
  Chassis chassis;
  DemaciaTrapezoid trapezoid;
  double vel;
  Translation2d error;
  public chackDriveng() {
    chassis = Chassis.getInstance();
    addRequirements(chassis);
    trapezoid = new DemaciaTrapezoid(2, 2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = new Translation2d(0, 1).minus(chassis.getPose().getTranslation());
    vel = trapezoid.nextVelocity(error.getNorm(), chassis.getChassisSpeedsVector().getNorm(), 0);
    chassis.setVelocities(new ChassisSpeeds(vel * error.getAngle().getCos(), vel * error.getAngle().getSin(), 0));
    LogManager.log("distenac left" + error.getNorm() + "vel" + vel);
    if(vel >= 0) LogManager.log("stop");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return error.getNorm() < 0.08;
  }
}
