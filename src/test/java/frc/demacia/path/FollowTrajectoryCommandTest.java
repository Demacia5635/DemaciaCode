package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Calls the command lifecycle directly, no scheduler. */
class FollowTrajectoryCommandTest {

    private static final double EPS = 1e-9;

    @BeforeAll
    static void initHal() {
        HAL.initialize(500, 0);
    }

    private final DemaciaTrajectory trajectory = TrajectoryBuilder.build(
            List.of(new Waypoint(0, 0, new Rotation2d()), new Waypoint(3, 0, new Rotation2d())));
    private final Pose2d pose = new Pose2d(1, 0, new Rotation2d());
    private final ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
    private ChassisSpeeds sent;

    private FollowTrajectoryCommand command() {
        return new FollowTrajectoryCommand(trajectory, () -> pose, () -> speeds, s -> sent = s);
    }

    @Test
    void executeSendsFollowerOutput() {
        FollowTrajectoryCommand command = command();
        PathFollower reference = new PathFollower();
        reference.start(trajectory);
        ChassisSpeeds expected = reference.calculate(pose, speeds);

        command.initialize();
        command.execute();

        assertEquals(expected.vxMetersPerSecond, sent.vxMetersPerSecond, EPS);
        assertEquals(expected.vyMetersPerSecond, sent.vyMetersPerSecond, EPS);
        assertEquals(expected.omegaRadiansPerSecond, sent.omegaRadiansPerSecond, EPS);
        assertTrue(sent.vxMetersPerSecond > 0);
        assertFalse(command.isFinished());
    }

    @Test
    void endSendsZeroSpeeds() {
        FollowTrajectoryCommand command = command();
        command.initialize();
        command.execute();

        command.end(false);

        assertEquals(0, sent.vxMetersPerSecond, EPS);
        assertEquals(0, sent.vyMetersPerSecond, EPS);
        assertEquals(0, sent.omegaRadiansPerSecond, EPS);
    }

    @Test
    void isFinishedOnceFollowerFinishes() {
        Pose2d atEnd = new Pose2d(3, 0, new Rotation2d());
        FollowTrajectoryCommand command = new FollowTrajectoryCommand(trajectory, () -> atEnd,
                () -> new ChassisSpeeds(), s -> sent = s);
        command.initialize();
        assertFalse(command.isFinished());
        command.execute();
        assertTrue(command.isFinished());
    }

    @Test
    void requiresTheGivenSubsystems() {
        Subsystem drive = new Subsystem() {};
        FollowTrajectoryCommand command = new FollowTrajectoryCommand(trajectory, () -> pose, () -> speeds, s -> sent = s,
                drive);
        assertTrue(command.getRequirements().contains(drive));
        assertEquals(1, command.getRequirements().size());
    }
}
