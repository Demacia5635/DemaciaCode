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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.LineSegment;
import frc.demacia.path.segments.BaseSegment;

class PathFollowerTest {

    private static final double EPS = 1e-9;

    @BeforeAll
    static void initHal() {
        HAL.initialize(500, 0);
    }

    private static Waypoint wp(double x, double y, double headingDeg) {
        return new Waypoint(x, y, Rotation2d.fromDegrees(headingDeg));
    }

    private static DemaciaTrajectory leftTurn() {
        return TrajectoryBuilder.build(List.of(wp(0, 0, 0), wp(10, 0, 0), wp(10, 10, 0)));
    }

    private static DemaciaTrajectory straight() {
        return TrajectoryBuilder.build(List.of(wp(0, 0, 0), wp(2, 0, 90)));
    }

    private static void assertVelocity(Translation2d expected, ChassisSpeeds actual) {
        assertEquals(expected.getX(), actual.vxMetersPerSecond, EPS);
        assertEquals(expected.getY(), actual.vyMetersPerSecond, EPS);
    }

    @Test
    void nonFiniteInputGivesZeroAndLeavesStateUntouched() {
        DemaciaTrajectory trajectory = leftTurn();
        PathFollower withNaN = new PathFollower();
        PathFollower clean = new PathFollower();
        withNaN.start(trajectory);
        clean.start(trajectory);
        ChassisSpeeds speeds = new ChassisSpeeds(2, 0, 0);
        Pose2d pose = new Pose2d(1, 0, new Rotation2d());

        clean.calculate(pose, speeds);
        withNaN.calculate(pose, speeds);
        ChassisSpeeds bad = withNaN.calculate(new Pose2d(Double.NaN, 0, new Rotation2d()), speeds);
        assertEquals(0, bad.vxMetersPerSecond, 0);
        assertEquals(0, bad.vyMetersPerSecond, 0);
        assertEquals(0, bad.omegaRadiansPerSecond, 0);
        ChassisSpeeds badSpeeds = withNaN.calculate(pose, new ChassisSpeeds(0, Double.NaN, 0));
        assertEquals(0, badSpeeds.vxMetersPerSecond, 0);

        Pose2d next = new Pose2d(1.1, 0, new Rotation2d());
        ChassisSpeeds a = withNaN.calculate(next, speeds);
        ChassisSpeeds b = clean.calculate(next, speeds);
        assertEquals(b.vxMetersPerSecond, a.vxMetersPerSecond, EPS);
        assertEquals(b.vyMetersPerSecond, a.vyMetersPerSecond, EPS);
        assertEquals(b.omegaRadiansPerSecond, a.omegaRadiansPerSecond, EPS);
    }

    @Test
    void nonFiniteInputOnTheFirstCycleLeavesNoTrace() {
        // a NaN first cycle after start() must change nothing, so the next good cycle
        // gives the same speeds as a follower that never saw the NaN
        DemaciaTrajectory trajectory = leftTurn();
        PathFollower withNaN = new PathFollower();
        PathFollower fresh = new PathFollower();
        withNaN.start(trajectory);
        fresh.start(trajectory);

        ChassisSpeeds bad = withNaN.calculate(new Pose2d(1, 0, Rotation2d.fromRadians(Double.NaN)),
                new ChassisSpeeds(1, 0, Double.NaN));
        assertEquals(0, bad.vxMetersPerSecond, 0);
        assertEquals(0, bad.vyMetersPerSecond, 0);
        assertEquals(0, bad.omegaRadiansPerSecond, 0);
        assertFalse(withNaN.isFinished());

        // heading 30 deg off and already turning, so any state left by the NaN cycle would change omega
        Pose2d pose = new Pose2d(1, 0, Rotation2d.fromDegrees(30));
        ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0.5);
        ChassisSpeeds a = withNaN.calculate(pose, speeds);
        ChassisSpeeds b = fresh.calculate(pose, speeds);
        assertEquals(b.vxMetersPerSecond, a.vxMetersPerSecond, EPS);
        assertEquals(b.vyMetersPerSecond, a.vyMetersPerSecond, EPS);
        assertEquals(b.omegaRadiansPerSecond, a.omegaRadiansPerSecond, EPS);
        assertTrue(Double.isFinite(a.omegaRadiansPerSecond));

        // and the cycle after that still matches
        Pose2d next = new Pose2d(1.02, 0, Rotation2d.fromDegrees(29));
        ChassisSpeeds a2 = withNaN.calculate(next, speeds);
        ChassisSpeeds b2 = fresh.calculate(next, speeds);
        assertEquals(b2.omegaRadiansPerSecond, a2.omegaRadiansPerSecond, EPS);
        assertEquals(b2.vxMetersPerSecond, a2.vxMetersPerSecond, EPS);
    }

    @Test
    void transitionCycleReturnsOutgoingSegmentCommandThenArcTakesOver() {
        DemaciaTrajectory trajectory = leftTurn();
        BaseSegment line = trajectory.getSegments().get(0);
        BaseSegment arc = trajectory.getSegments().get(1);
        assertTrue(line instanceof LineSegment);
        assertTrue(arc instanceof ArcSegment);

        // 7 cm from the end of the line: the line is finished, the arc is not
        Translation2d position = line.getEndPose().getTranslation().plus(new Translation2d(-0.05, 0.05));
        Pose2d pose = new Pose2d(position, new Rotation2d());
        ChassisSpeeds speeds = new ChassisSpeeds(3, 0, 0);

        PathFollower follower = new PathFollower();
        follower.start(trajectory);

        ChassisSpeeds transition = follower.calculate(pose, speeds);
        assertVelocity(line.getVelocity(position, new Translation2d(3, 0)), transition);

        ChassisSpeeds next = follower.calculate(pose, speeds);
        assertVelocity(arc.getVelocity(position, new Translation2d(3, 0)), next);
        assertFalse(follower.isFinished());
    }

    @Test
    void finalPositionWithHeadingErrorStopsTranslationAndKeepsTurning() {
        DemaciaTrajectory trajectory = straight();
        PathFollower follower = new PathFollower();
        follower.start(trajectory);

        Pose2d atEnd = new Pose2d(2, 0, new Rotation2d()); // 90 deg off
        ChassisSpeeds out = follower.calculate(atEnd, new ChassisSpeeds());

        assertEquals(0, out.vxMetersPerSecond, EPS);
        assertEquals(0, out.vyMetersPerSecond, EPS);
        assertTrue(out.omegaRadiansPerSecond > 0, "turns towards +90 deg");
        assertFalse(follower.isFinished());
    }

    @Test
    void stopPointWaitsUntilSpeedIsAtMostTheStoppedSpeed() {
        Pose2d atEnd = new Pose2d(2, 0, Rotation2d.fromDegrees(90)); // heading already right
        double stopped = PathConstants.MAX_VELOCITY_TO_CONSIDER_STOPPED;

        PathFollower moving = new PathFollower();
        moving.start(straight());
        ChassisSpeeds out = moving.calculate(atEnd, new ChassisSpeeds(stopped + 1e-6, 0, 0));
        assertEquals(0, out.vxMetersPerSecond, EPS);
        assertEquals(0, out.vyMetersPerSecond, EPS);
        assertFalse(moving.isFinished());

        PathFollower slow = new PathFollower();
        slow.start(straight());
        slow.calculate(atEnd, new ChassisSpeeds(stopped, 0, 0)); // exactly the limit counts as stopped (oldPath)
        assertTrue(slow.isFinished());
    }

    @Test
    void headingErrorOfExactlyTheToleranceIsNotFinished() {
        // tolerance 90 deg, end heading 90 deg, robot heading 0: error is exactly the tolerance, strict < like oldPath
        PathConfig config = new PathConfig().withFinalHeadingTolerance(Math.PI / 2);
        PathFollower follower = new PathFollower();
        follower.start(TrajectoryBuilder.build(List.of(wp(0, 0, 0), wp(2, 0, 90)), config));
        follower.calculate(new Pose2d(2, 0, new Rotation2d()), new ChassisSpeeds());
        assertFalse(follower.isFinished());
    }

    @Test
    void afterFinishingSendsZeroEvenIfPushedAway() {
        PathFollower follower = new PathFollower();
        follower.start(straight());
        follower.calculate(new Pose2d(2, 0, Rotation2d.fromDegrees(90)), new ChassisSpeeds());
        assertTrue(follower.isFinished());

        ChassisSpeeds out = follower.calculate(new Pose2d(0, 1, new Rotation2d()), new ChassisSpeeds(1, 0, 0));
        assertEquals(0, out.vxMetersPerSecond, EPS);
        assertEquals(0, out.vyMetersPerSecond, EPS);
        assertEquals(0, out.omegaRadiansPerSecond, EPS);
        assertTrue(follower.isFinished());
    }

    @Test
    void headingWithinToleranceFinishesWithZeroSpeeds() {
        PathFollower follower = new PathFollower();
        follower.start(straight());

        Pose2d atEnd = new Pose2d(2, 0, Rotation2d.fromDegrees(88));
        ChassisSpeeds out = follower.calculate(atEnd, new ChassisSpeeds());

        assertEquals(0, out.vxMetersPerSecond, EPS);
        assertEquals(0, out.vyMetersPerSecond, EPS);
        assertEquals(0, out.omegaRadiansPerSecond, EPS);
        assertTrue(follower.isFinished());
    }

    @Test
    void restartAfterFinishBeginsAtFirstSegment() {
        DemaciaTrajectory trajectory = straight();
        PathFollower follower = new PathFollower();
        follower.start(trajectory);
        follower.calculate(new Pose2d(2, 0, Rotation2d.fromDegrees(90)), new ChassisSpeeds());
        assertTrue(follower.isFinished());

        follower.start(trajectory);
        assertFalse(follower.isFinished());
        ChassisSpeeds out = follower.calculate(new Pose2d(0, 0, new Rotation2d()), new ChassisSpeeds());
        assertVelocity(trajectory.getSegments().get(0).getVelocity(new Translation2d(), new Translation2d()), out);
        assertTrue(out.vxMetersPerSecond > 0);
    }

    @Test
    void restartMidPathResetsSegmentIndex() {
        DemaciaTrajectory trajectory = leftTurn();
        BaseSegment line = trajectory.getSegments().get(0);
        Translation2d nearLineEnd = line.getEndPose().getTranslation().plus(new Translation2d(-0.05, 0));
        PathFollower follower = new PathFollower();
        follower.start(trajectory);
        follower.calculate(new Pose2d(nearLineEnd, new Rotation2d()), new ChassisSpeeds()); // moves on to the arc

        follower.start(trajectory);
        ChassisSpeeds out = follower.calculate(new Pose2d(), new ChassisSpeeds());
        assertVelocity(line.getVelocity(new Translation2d(), new Translation2d()), out);
    }

    @Test
    void emptyPathIsFinishedImmediately() {
        PathFollower follower = new PathFollower();
        follower.start(TrajectoryBuilder.build(List.of()));
        assertTrue(follower.isFinished());
    }

    @Test
    void firstCycleFromRestUsesConfiguredAccel() {
        PathConfig config = new PathConfig().withMaxAccel(1);
        PathFollower follower = new PathFollower();
        follower.start(TrajectoryBuilder.build(List.of(wp(0, 0, 0), wp(10, 0, 0)), config));

        ChassisSpeeds out = follower.calculate(new Pose2d(), new ChassisSpeeds());

        assertEquals(1 * PathConstants.DT, out.vxMetersPerSecond, EPS); // not the default 3 * 0.02
        assertEquals(0, out.vyMetersPerSecond, EPS);
    }
}
