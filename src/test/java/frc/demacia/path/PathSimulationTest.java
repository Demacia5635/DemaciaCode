package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.commands.FollowTrajectoryCommand;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.RobotContainer;

/**
 * Runs the real robot code (RobotContainer, Chassis, RobotPose, FollowTrajectoryCommand) in simulation
 * and checks that the robot really drives through every waypoint.
 *
 * <p>Two poses are kept every loop:
 * <ul>
 *   <li><b>sim world</b>: where the robot really is. Built only from what the simulated wheels did
 *       (wheel speed + wheel angle), with WPILib kinematics.</li>
 *   <li><b>code thinks</b>: {@link RobotPose#getEstimatedPose()}, which the path follower uses.</li>
 * </ul>
 *
 * Run with: {@code ./gradlew test --tests frc.demacia.path.PathSimulationTest}
 * The full run is saved to {@code build/path-sim.csv}.
 */
class PathSimulationTest {

    private static final double LOOP_SECONDS = 0.02;
    private static final double TIMEOUT_SECONDS = 30;

    /** The sim world pose must get this close to every waypoint, in order. */
    private static final double WAYPOINT_TOLERANCE_METERS = 0.15;
    /** Final sim world pose vs. last waypoint. */
    private static final double FINAL_POSITION_TOLERANCE_METERS = 0.1;
    private static final double FINAL_HEADING_TOLERANCE_DEG = 5;
    /** Code thinks vs. sim world, on every loop. */
    private static final double MAX_POSITION_ERROR_METERS = 0.05;
    private static final double MAX_HEADING_ERROR_DEG = 2;
    /**
     * In simulation the sensors reach RobotPose 2 loops (40 ms) late: the motor position is moved in
     * Log.periodic, then read by RobotPose in the next loop. So "code thinks" is compared with the
     * sim world pose from 2 loops before. (Measured with a straight drive: the gap is exactly 2 loops
     * while moving, and 0 when stopped.)
     */
    private static final int SENSOR_DELAY_LOOPS = 2;

    private record Sample(double time, Pose2d simWorld, Pose2d codeThinks) {}

    @Test
    void robotDrivesThroughAllWaypoints() throws Exception {
        assertTrue(HAL.initialize(500, 0), "HAL failed to start");
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        RobotContainer robotContainer = new RobotContainer();
        List<Pose2d> waypoints = robotContainer.points;
        Chassis chassis = Chassis.getInstance();
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(chassis.getModuleLocations());

        // Let the simulated devices start up before driving
        for (int i = 0; i < 25; i++) {
            runOneLoop();
        }

        DemaciaTrajectory trajectory = new DemaciaTrajectory(waypoints);
        FollowTrajectoryCommand command = new FollowTrajectoryCommand(trajectory, RobotContainer.driveCommand);
        CommandScheduler.getInstance().schedule(command);

        Pose2d simWorld = waypoints.get(0);
        List<Sample> samples = new ArrayList<>();
        double time = 0;

        while (!trajectory.isFinished() && time < TIMEOUT_SECONDS) {
            runOneLoop();
            time += LOOP_SECONDS;

            // Move the sim world pose by what the wheels did in this loop
            ChassisSpeeds wheelSpeeds = kinematics.toChassisSpeeds(chassis.getModuleStates());
            simWorld = simWorld.exp(new Twist2d(
                    wheelSpeeds.vxMetersPerSecond * LOOP_SECONDS,
                    wheelSpeeds.vyMetersPerSecond * LOOP_SECONDS,
                    wheelSpeeds.omegaRadiansPerSecond * LOOP_SECONDS));

            samples.add(new Sample(time, simWorld, RobotPose.getInstance().getEstimatedPose()));
        }
        // One more loop so the command ends and the chassis stops
        runOneLoop();

        saveCsv(samples);
        boolean passed = checkAndPrint(waypoints, samples, trajectory.isFinished(), time);
        assertTrue(passed, "Path simulation failed, see the table above and build/path-sim.csv");
    }

    /** One robot loop: what Robot.robotPeriodic does, then wait like a real 20 ms loop. */
    private static void runOneLoop() throws InterruptedException {
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().run();
        // Real time wait, so the Phoenix simulation (which runs in real time) keeps up
        Thread.sleep((long) (LOOP_SECONDS * 1000));
    }

    // ---------------------------------------------------------------- checks

    private static boolean checkAndPrint(List<Pose2d> waypoints, List<Sample> samples, boolean isFinished,
            double time) {
        boolean allPassed = true;
        StringBuilder report = new StringBuilder("\n========== PATH SIMULATION ==========\n");

        // Check 1: the path finishes
        report.append(String.format("1. Path finished: %s after %.2f s   %s%n",
                isFinished, time, passFail(isFinished)));
        allPassed &= isFinished;

        // Check 2: the sim world pose passes through every waypoint, in order
        report.append("2. Waypoints (sim world must get within ")
                .append(WAYPOINT_TOLERANCE_METERS).append(" m, in order)\n");
        report.append(String.format("   %-16s %-22s %-10s %-8s %s%n",
                "waypoint", "closest sim pose", "distance", "time", "result"));
        int searchFrom = 0;
        for (Pose2d waypoint : waypoints) {
            int closestIndex = -1;
            double closestDistance = Double.MAX_VALUE;
            for (int i = searchFrom; i < samples.size(); i++) {
                double distance = samples.get(i).simWorld().getTranslation().getDistance(waypoint.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestIndex = i;
                }
                // Stop at the first time it is reached, so the next waypoint is searched after this one
                if (distance < WAYPOINT_TOLERANCE_METERS) {
                    break;
                }
            }
            boolean reached = closestDistance < WAYPOINT_TOLERANCE_METERS;
            allPassed &= reached;
            if (closestIndex >= 0) {
                Sample closest = samples.get(closestIndex);
                report.append(String.format("   %-16s %-22s %-10s %-8s %s%n",
                        xy(waypoint), xy(closest.simWorld()),
                        String.format("%.3f m", closestDistance),
                        String.format("%.2f s", closest.time()), passFail(reached)));
                if (reached) {
                    searchFrom = closestIndex;
                }
            } else {
                report.append("   ").append(xy(waypoint)).append("  no samples   FAIL\n");
            }
        }

        // Check 3: final pose
        Pose2d lastWaypoint = waypoints.get(waypoints.size() - 1);
        Pose2d finalSimPose = samples.isEmpty() ? waypoints.get(0) : samples.get(samples.size() - 1).simWorld();
        double finalDistance = finalSimPose.getTranslation().getDistance(lastWaypoint.getTranslation());
        double finalHeadingErrorDeg = Math.abs(finalSimPose.getRotation().minus(lastWaypoint.getRotation()).getDegrees());
        boolean finalOk = finalDistance < FINAL_POSITION_TOLERANCE_METERS
                && finalHeadingErrorDeg < FINAL_HEADING_TOLERANCE_DEG;
        allPassed &= finalOk;
        report.append(String.format("3. Final sim pose %s, %.1f deg: %.3f m and %.1f deg from last waypoint   %s%n",
                xy(finalSimPose), finalSimPose.getRotation().getDegrees(), finalDistance, finalHeadingErrorDeg,
                passFail(finalOk)));

        // Check 4: code thinks vs. sim world
        double maxPositionError = 0;
        double maxHeadingErrorDeg = 0;
        double maxPositionErrorTime = 0;
        double maxHeadingErrorTime = 0;
        for (int i = SENSOR_DELAY_LOOPS; i < samples.size(); i++) {
            Pose2d codeThinks = samples.get(i).codeThinks();
            Pose2d simWorld = samples.get(i - SENSOR_DELAY_LOOPS).simWorld();
            double positionError = codeThinks.getTranslation().getDistance(simWorld.getTranslation());
            double headingErrorDeg = Math.abs(codeThinks.getRotation().minus(simWorld.getRotation()).getDegrees());
            if (positionError > maxPositionError) {
                maxPositionError = positionError;
                maxPositionErrorTime = samples.get(i).time();
            }
            if (headingErrorDeg > maxHeadingErrorDeg) {
                maxHeadingErrorDeg = headingErrorDeg;
                maxHeadingErrorTime = samples.get(i).time();
            }
        }
        boolean agreeOk = maxPositionError < MAX_POSITION_ERROR_METERS && maxHeadingErrorDeg < MAX_HEADING_ERROR_DEG;
        allPassed &= agreeOk;
        report.append(String.format(
                "4. Max error code-thinks vs sim-world (%d loops before): %.3f m (at %.2f s), %.2f deg (at %.2f s)   %s%n",
                SENSOR_DELAY_LOOPS, maxPositionError, maxPositionErrorTime, maxHeadingErrorDeg, maxHeadingErrorTime,
                passFail(agreeOk)));

        report.append("RESULT: ").append(passFail(allPassed)).append('\n');
        report.append("=====================================\n");
        System.out.println(report);
        return allPassed;
    }

    private static String passFail(boolean ok) {
        return ok ? "PASS" : "FAIL";
    }

    private static String xy(Pose2d pose) {
        return String.format("(%.2f, %.2f)", pose.getX(), pose.getY());
    }

    private static void saveCsv(List<Sample> samples) throws IOException {
        Path file = Path.of("build", "path-sim.csv");
        Files.createDirectories(file.getParent());
        try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
            out.println("time,sim_x,sim_y,sim_deg,code_x,code_y,code_deg");
            for (Sample s : samples) {
                out.printf("%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f%n", s.time(),
                        s.simWorld().getX(), s.simWorld().getY(), s.simWorld().getRotation().getDegrees(),
                        s.codeThinks().getX(), s.codeThinks().getY(), s.codeThinks().getRotation().getDegrees());
            }
        }
    }
}
