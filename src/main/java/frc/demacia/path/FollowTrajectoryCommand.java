package frc.demacia.path;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that drives the robot along a DemaciaTrajectory.
 * Every 20 ms it asks a PathFollower what speeds to drive and sends them to the chassis.
 *
 * <p>It does NOT reset the robot pose. Reset the pose once at the start of your auto,
 * so chained paths keep the real odometry and vision corrections.
 * Add {@code .withTimeout()} in autos, because the follower has no timeout of its own.
 *
 * <pre>{@code
 * DemaciaTrajectory path = TrajectoryBuilder.build(List.of(new Waypoint(startPose), new Waypoint(scorePose)));
 * Command drive = new FollowTrajectoryCommand(path,
 *         RobotPose.getInstance()::getEstimatedPose,  // field pose
 *         chassis::getChassisSpeedsFieldRel,          // measured field-relative speeds
 *         driveCommand::setVelFieldRel,               // field-relative output
 *         chassis).withTimeout(5);
 * }</pre>
 */
public class FollowTrajectoryCommand extends Command {
    private final DemaciaTrajectory trajectory;
    private final PathFollower follower = new PathFollower();
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private final Consumer<ChassisSpeeds> fieldSpeedsConsumer;

    /**
     * @param trajectory the route to drive (from TrajectoryBuilder)
     * @param poseSupplier gives the robot pose on the field (meters and rotation)
     * @param fieldSpeeds gives the measured speeds, field-relative (m/s, m/s, rad/s)
     * @param fieldSpeedsConsumer receives the wanted speeds, field-relative (m/s, m/s, rad/s)
     * @param requirements the subsystems this command uses (normally the chassis)
     */
    public FollowTrajectoryCommand(DemaciaTrajectory trajectory, Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeeds, Consumer<ChassisSpeeds> fieldSpeedsConsumer, Subsystem... requirements) {
        this.trajectory = trajectory;
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeeds;
        this.fieldSpeedsConsumer = fieldSpeedsConsumer;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        follower.start(trajectory);
    }

    @Override
    public void execute() {
        fieldSpeedsConsumer.accept(follower.calculate(poseSupplier.get(), fieldSpeedsSupplier.get()));
    }

    // always stop the robot, whether we finished or were interrupted
    @Override
    public void end(boolean interrupted) {
        fieldSpeedsConsumer.accept(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }
}
