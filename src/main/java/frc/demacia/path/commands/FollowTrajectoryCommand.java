package frc.demacia.path.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.DemaciaTrajectory;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.wpilibSwerve.command.SwerveCommand;

/** Drives the chassis along a {@link DemaciaTrajectory} until it is finished. */
public class FollowTrajectoryCommand extends Command {

    private final DemaciaTrajectory trajectory;
    private final Chassis chassis;
    private final SwerveCommand swerveCommand;

    public FollowTrajectoryCommand(DemaciaTrajectory trajectory, SwerveCommand swerveCommand) {
        this.chassis = Chassis.getInstance();
        this.trajectory = trajectory;
        this.swerveCommand = swerveCommand;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        swerveCommand.odometry.resetPose(Pose2d.kZero);
        trajectory.start();
    }

    @Override
    public void execute() {
        ChassisSpeeds wantedSpeeds = trajectory.calculateChassisSpeeds(
                chassis.getChassisSpeedsFieldRel(), RobotPose.getInstance().getEstimatedPose());
        swerveCommand.setVelFieldRel(wantedSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    @Override
    public boolean isFinished() {
        return trajectory.isFinished();
    }
}
