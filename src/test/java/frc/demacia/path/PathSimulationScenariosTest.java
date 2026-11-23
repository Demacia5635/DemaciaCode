package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.BiConsumer;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.demacia.path.PathSimulationHarness.Cmd;
import frc.demacia.path.PathSimulationHarness.Run;
import frc.demacia.path.PathSimulationHarness.Sample;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.BaseSegment;

/**
 * Full-robot simulation scenarios for the path follower (same robot code as {@link PathSimulationTest}).
 * Every scenario writes {@code build/path-sim-scenario-<name>.csv}; the summary table is printed at
 * the end and saved to {@code build/path-sim-scenarios-summary.csv}.
 *
 * <p>Standard pass criteria (unless a scenario says otherwise):
 * <ul>
 *   <li>finishes before a harness timeout (the follower has none of its own);</li>
 *   <li>no NaN/Infinity ever reaches the speeds consumer;</li>
 *   <li>commanded translation speed never above the config maxSpeed;</li>
 *   <li>no command above measured speed + maxAccel * 0.02 (the trapezoid's rule, also checks the
 *       config's maxAccel really reached the segments);</li>
 *   <li>code pose (what the follower controls) ends within finalPositionThreshold and
 *       finalHeadingTolerance of the last waypoint;</li>
 *   <li>sim world (truth) ends within 0.15 m and 7 deg. ASSUMPTION: 0.05 m / 2 deg above the follower's
 *       own thresholds, the most estimator vs sim world may differ (PathSimulationTest check 4 limits:
 *       0.05 m, 2 deg; the measured value is printed per scenario as "estimator vs sim");</li>
 *   <li>sim world stays within 0.15 m of the built segments (lines and arcs), the same number as
 *       PathSimulationTest's waypoint tolerance.</li>
 * </ul>
 * Result column: PASS; FAIL (fails the test); KNOWN (only checks marked as an oldPath-preserved quirk
 * failed, verified against git HEAD oldPath; the test passes but the quirk is printed, never hidden);
 * PROBE (report-only experiment with deliberately bad input, never fails).
 *
 * <p>Raw command accel (delta v / 0.02) is reported, not asserted: oldPath's trapezoid jumps to the end
 * speed when under 0.08 s of braking is left, and a stop point sends zero until measured speed is
 * under 0.1 m/s, so step changes are part of the kept design.
 */
class PathSimulationScenariosTest {

    private static final PathConfig DEFAULT = new PathConfig();
    private static final double TIMEOUT = 20;
    private static final double PATH_DEVIATION_LIMIT = 0.15;
    private static final double TRUTH_FINAL_POSITION_LIMIT = 0.15;
    private static final double TRUTH_FINAL_HEADING_LIMIT_DEG = 7;

    private static final List<String> SUMMARY = new ArrayList<>();

    @BeforeAll
    static void startRobot() throws Exception {
        PathSimulationHarness.start();
    }

    @AfterAll
    static void printSummary() throws Exception {
        PathSimulationHarness.restore();
        StringBuilder sb = new StringBuilder("\n================ PATH SIM SCENARIOS ================\n");
        sb.append(String.format("%-28s %-6s %s%n", "scenario", "result", "key numbers"));
        for (String row : SUMMARY) {
            sb.append(row).append('\n');
        }
        System.out.println(sb);
        Path file = Path.of("build", "path-sim-scenarios-summary.csv");
        Files.createDirectories(file.getParent());
        try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
            out.println("scenario,result,key numbers");
            for (String row : SUMMARY) {
                String[] parts = row.trim().split("\\s+", 3);
                out.println(parts[0] + "," + parts[1] + ",\"" + (parts.length > 2 ? parts[2] : "") + "\"");
            }
        }
    }

    // ===================================================================== report helper

    private static final class Report {
        final String name;
        final List<String> lines = new ArrayList<>();
        final List<String> failed = new ArrayList<>();
        final List<String> knownFailed = new ArrayList<>();
        final StringBuilder key = new StringBuilder();
        final boolean probe;

        Report(String name, boolean probe) {
            this.name = name;
            this.probe = probe;
        }

        Report(String name) {
            this(name, false);
        }

        void check(boolean ok, String format, Object... args) {
            String text = String.format(format, args);
            lines.add((ok ? "  PASS  " : "  FAIL  ") + text);
            if (!ok) {
                failed.add(text);
            }
        }

        /**
         * A check whose failure is a KNOWN behavior kept from oldPath on purpose (verified against
         * git HEAD oldPath). It does not fail the test, but the scenario result becomes KNOWN, not PASS.
         */
        void known(boolean ok, String format, Object... args) {
            String text = String.format(format, args);
            lines.add((ok ? "  PASS  " : "  KNOWN ") + text);
            if (!ok) {
                knownFailed.add(text);
            }
        }

        void note(String format, Object... args) {
            lines.add("  info  " + String.format(format, args));
        }

        void key(String format, Object... args) {
            if (key.length() > 0) {
                key.append("; ");
            }
            key.append(String.format(format, args));
        }

        void finish() {
            String result = probe ? "PROBE" : !failed.isEmpty() ? "FAIL" : !knownFailed.isEmpty() ? "KNOWN" : "PASS";
            StringBuilder sb = new StringBuilder("\n---- " + name + " : " + result + "\n");
            lines.forEach(l -> sb.append(l).append('\n'));
            System.out.println(sb);
            SUMMARY.add(String.format("%-28s %-6s %s", name, result, key));
            if (!probe) {
                assertTrue(failed.isEmpty(), name + " failed: " + String.join(" | ", failed));
            }
        }
    }

    // ===================================================================== building blocks

    private static Waypoint wp(double x, double y, double deg) {
        return new Waypoint(x, y, Rotation2d.fromDegrees(deg));
    }

    private static Pose2d pose(double x, double y, double deg) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
    }

    /** Teleports to start, builds, runs one FollowTrajectoryCommand. */
    private static Run runPath(String name, Pose2d start, List<Waypoint> points, PathConfig config,
            UnaryOperator<Pose2d> poseFilter, UnaryOperator<ChassisSpeeds> speedFilter,
            BiConsumer<Integer, Command> hook, double timeout) throws Exception {
        PathSimulationHarness.teleport(start);
        Run run = new Run(name);
        DemaciaTrajectory trajectory = TrajectoryBuilder.build(points, config);
        FollowTrajectoryCommand command = run.follow(trajectory, points, poseFilter, speedFilter);
        run.execute(command, timeout, hook);
        run.saveCsv();
        return run;
    }

    private static Run runPath(String name, Pose2d start, List<Waypoint> points, PathConfig config) throws Exception {
        return runPath(name, start, points, config, null, null, null, TIMEOUT);
    }

    private static Run runPath(String name, List<Waypoint> points) throws Exception {
        return runPath(name, points.get(0), points, DEFAULT);
    }

    /** Metrics every scenario records, plus the standard checks. devFrom: path deviation checked from this time on. */
    private static void standardChecks(Report r, Run run, PathConfig config, Pose2d target, double devFrom) {
        standardChecks(r, run, config, target, devFrom, false);
    }

    private static void standardChecks(Report r, Run run, PathConfig config, Pose2d target, double devFrom,
            boolean deviationIsKnownQuirk) {
        double[] accel = run.cmdAccel();
        r.check(run.ended && !run.timedOut, "finished: %s after %.2f s (timeout %s)", run.ended && !run.timedOut,
                run.duration, run.timedOut ? "HIT" : "not hit");
        r.check(run.nonFiniteCommands == 0, "non-finite commands: %d of %d", run.nonFiniteCommands, run.cmds.size());
        r.check(run.syncFailures == 0, "harness determinism: Phoenix signal sync failures %d (last %s)",
                run.syncFailures, PathSimulationHarness.lastSyncStatus);
        r.check(run.maxCmdSpeed() <= config.maxSpeed + 1e-9, "max commanded speed %.3f <= maxSpeed %.2f",
                run.maxCmdSpeed(), config.maxSpeed);
        int violations = run.accelRuleViolations(config.maxAccel);
        r.check(violations == 0, "commands above measured + maxAccel*DT (%.2f m/s^2): %d", config.maxAccel, violations);
        // the follower's own finish rule is on the code pose of the cycle it finished, so check that one;
        // after the robot settles the estimator can still move by ~0.01 m (sensor delay), noted below
        Pose2d atFinish = run.samples.isEmpty() ? run.finalCode : run.samples.get(run.samples.size() - 1).code();
        double codePos = atFinish.getTranslation().getDistance(target.getTranslation());
        double codeDeg = Run.headingErrorDeg(atFinish, target);
        double tolDeg = Math.toDegrees(config.finalHeadingTolerance);
        if (run.lastSeenPose != null && (run.poseNoiseAllowanceM > 0 || run.poseNoiseAllowanceDeg > 0)) {
            // the follower can only keep its tolerance on the pose it is given: check that strictly, and the
            // un-noised estimate with the 3 sigma of the noise this test injected on top
            double seenPos = run.lastSeenPose.getTranslation().getDistance(target.getTranslation());
            double seenDeg = Run.headingErrorDeg(run.lastSeenPose, target);
            r.check(seenPos < config.finalPositionThreshold && seenDeg < tolDeg,
                    "pose the follower was given when it finished %.3f m, %.2f deg (limits %.2f m, %.1f deg)",
                    seenPos, seenDeg, config.finalPositionThreshold, tolDeg);
            r.check(codePos < config.finalPositionThreshold + run.poseNoiseAllowanceM
                    && codeDeg < tolDeg + run.poseNoiseAllowanceDeg,
                    "un-noised code pose when it finished %.3f m, %.2f deg (limits %.2f + %.2f m, %.1f + %.1f deg noise)",
                    codePos, codeDeg, config.finalPositionThreshold, run.poseNoiseAllowanceM, tolDeg,
                    run.poseNoiseAllowanceDeg);
        } else {
            r.check(codePos < config.finalPositionThreshold && codeDeg < tolDeg,
                    "code pose when it finished %.3f m, %.2f deg from target (limits %.2f m, %.1f deg)", codePos,
                    codeDeg, config.finalPositionThreshold, tolDeg);
        }
        r.note("code pose after settling %.3f m, %.2f deg from target",
                run.finalCode.getTranslation().getDistance(target.getTranslation()), Run.headingErrorDeg(run.finalCode, target));
        double simPos = run.finalSim.getTranslation().getDistance(target.getTranslation());
        double simDeg = Run.headingErrorDeg(run.finalSim, target);
        r.check(simPos < TRUTH_FINAL_POSITION_LIMIT && simDeg < TRUTH_FINAL_HEADING_LIMIT_DEG,
                "final sim world %.3f m, %.2f deg from target (limits %.2f m, %.0f deg)", simPos, simDeg,
                TRUTH_FINAL_POSITION_LIMIT, TRUTH_FINAL_HEADING_LIMIT_DEG);
        double dev = run.maxPathDeviation(devFrom, Double.MAX_VALUE);
        (deviationIsKnownQuirk ? (Checker) r::known : (Checker) r::check).apply(dev < PATH_DEVIATION_LIMIT,
                "max sim distance from built path%s: %.3f m (limit %.2f)",
                devFrom > 0 ? String.format(" after %.2f s", devFrom) : "", dev, PATH_DEVIATION_LIMIT);
        r.note("max distance from waypoint polyline %.3f m, max sim speed %.3f, max |omega| cmd %.2f rad/s",
                run.maxPolylineDeviation(), run.maxSimSpeed(), run.maxCmdOmega());
        r.note("cmd accel: max speed-up %.2f, max slow-down %.2f, max vector %.2f m/s^2 (maxAccel %.2f); "
                + "max true accel over 0.1 s %.2f", accel[0], accel[1], accel[2], config.maxAccel, run.maxSimAccel());
        r.note("estimator vs sim (2 loops) max %.3f m; Rotation2d zero-vector errors %d",
                run.maxEstimatorError(), run.rotationZeroErrors);
        r.key("%s %.2fs dev %.3f fin %.3fm/%.1fdeg vmax %.2f/%.1f acc+%.1f/-%.1f NaN %d", run.ended && !run.timedOut
                ? "done" : "HANG", run.duration, run.maxPathDeviation(), simPos, simDeg, run.maxCmdSpeed(),
                config.maxSpeed, accel[0], accel[1], run.nonFiniteCommands);
    }

    private static void standardChecks(Report r, Run run, PathConfig config, Pose2d target) {
        standardChecks(r, run, config, target, -1);
    }

    private interface Checker {
        void apply(boolean ok, String format, Object... args);
    }

    private static Pose2d last(List<? extends Pose2d> points) {
        return points.get(points.size() - 1);
    }

    private static int arcIndex(DemaciaTrajectory t) {
        List<BaseSegment> s = t.getSegments();
        for (int i = 0; i < s.size(); i++) {
            if (s.get(i) instanceof ArcSegment) {
                return i;
            }
        }
        return -1;
    }

    // ===================================================================== 1. scenarios

    @Test
    void baselineRoute() throws Exception {
        // the RobotContainer route, as a reference for the deviation numbers
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(2, -2, 90), wp(2, 0, 180), wp(1.5, -1, 270), wp(0, 0, 0));
        Run run = runPath("baseline-route", points);
        Report r = new Report("baseline-route");
        standardChecks(r, run, DEFAULT, last(points));
        r.finish();
    }

    private static Run turn(String name, double turnDeg) throws Exception {
        double a = Math.toRadians(turnDeg);
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(3, 0, 0), wp(3 + 3 * Math.cos(a), 3 * Math.sin(a), 0));
        Run run = runPath(name, points);
        Report r = new Report(name);
        standardChecks(r, run, DEFAULT, last(points));
        r.note("segments: %s", TrajectoryBuilder.build(points).getSegments().size());
        r.finish();
        return run;
    }

    @Test
    void sharpTurn90BothDirections() throws Exception {
        Run left = turn("turn90-left", 90);
        Run right = turn("turn90-right", -90);
        // a frame or sign mistake shows up as an asymmetric result
        Report r = new Report("turn90-symmetry");
        double devDiff = Math.abs(left.maxPathDeviation() - right.maxPathDeviation());
        double timeDiff = Math.abs(left.duration - right.duration);
        r.check(devDiff < 0.03, "left vs right max path deviation differ by %.3f m (limit 0.03)", devDiff);
        r.check(timeDiff <= 0.1 + 1e-9, "left vs right time differ by %.2f s (limit 0.10)", timeDiff);
        r.key("dev L %.3f R %.3f, time L %.2f R %.2f", left.maxPathDeviation(), right.maxPathDeviation(),
                left.duration, right.duration);
        r.finish();
    }

    @Test
    void sharpTurn150() throws Exception {
        turn("turn150", 150);
    }

    @Test
    void midPathStopPoint() throws Exception {
        // heading changes into the stop, so the robot may still rotate while it stops
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(2, 0, 90).withMaxCornerSpeed(0), wp(2, 2, 90));
        Run run = runPath("mid-stop", points);
        Report r = new Report("mid-stop");
        standardChecks(r, run, DEFAULT, last(points));
        Translation2d stop = points.get(1).getTranslation();
        // truth: stopped near the stop point before going on
        double minSpeedNear = Double.MAX_VALUE;
        double stoppedTime = 0;
        for (Sample s : run.samples) {
            if (s.sim().getTranslation().getDistance(stop) < 0.15) {
                minSpeedNear = Math.min(minSpeedNear, s.simSpeed());
                if (s.simSpeed() < 0.1) {
                    stoppedTime += PathSimulationHarness.DT;
                }
            }
        }
        r.check(minSpeedNear < 0.1, "min true speed within 0.15 m of stop point: %.3f m/s (< 0.1)", minSpeedNear);
        // follower: left the stop segment only with measured speed < 0.1
        double seenAtLeave = Double.NaN;
        for (int i = 1; i < run.cmds.size(); i++) {
            if (run.cmds.get(i - 1).segment() == 0 && run.cmds.get(i).segment() == 1) {
                seenAtLeave = run.cmds.get(i - 1).measuredSeen();
            }
        }
        r.check(seenAtLeave < 0.1, "measured speed on the cycle it left the stop: %.3f m/s (< 0.1)", seenAtLeave);
        r.note("time with true speed < 0.1 near the stop: %.2f s; max |measured - commanded speed| one loop later %.3f m/s",
                stoppedTime, run.maxSpeedRoundTripError()[0]);
        r.key("min v near stop %.3f, left at measured %.3f, still %.2fs", minSpeedNear, seenAtLeave, stoppedTime);
        r.finish();
    }

    @Test
    void exactReversal() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(2, 0, 0), wp(0, 0, 0));
        Run run = runPath("reversal-180", points);
        Report r = new Report("reversal-180");
        standardChecks(r, run, DEFAULT, last(points));
        double minNear = run.samples.stream().filter(s -> s.sim().getTranslation().getDistance(new Translation2d(2, 0)) < 0.15)
                .mapToDouble(Sample::simSpeed).min().orElse(Double.MAX_VALUE);
        r.check(minNear < 0.1, "a 180 deg reversal becomes a stop: min true speed near (2,0) %.3f m/s", minNear);
        r.finish();
    }

    @Test
    void nearReversal178() throws Exception {
        // 178 deg is under the 179 deg reversal limit, so it gets an arc that must pass through (2,0)
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(2, 0, 0), wp(0, 0.07, 0));
        Run run = runPath("near-reversal-178", points);
        Report r = new Report("near-reversal-178");
        standardChecks(r, run, DEFAULT, last(points));
        DemaciaTrajectory t = TrajectoryBuilder.build(points);
        int arc = arcIndex(t);
        r.note("arc kept: %s%s", arc >= 0, arc >= 0 ? String.format(", radius %.3f m, speed %.2f m/s",
                ((ArcSegment) t.getSegments().get(arc)).getRadius(), t.getSegments().get(arc).getMaxVelocity()) : "");
        r.note("KNOWN (oldPath-preserved): the loop swings %.3f m off the straight waypoint lines", run.maxPolylineDeviation());
        r.key("polyline dev %.3f m", run.maxPolylineDeviation());
        r.finish();
    }

    @Test
    void slowCornerCap() throws Exception {
        double cap = 1.0;
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(4, 0, 0).withMaxCornerSpeed(cap), wp(4, 4, 0));
        Run run = runPath("slow-corner-1mps", points);
        Report r = new Report("slow-corner-1mps");
        standardChecks(r, run, DEFAULT, last(points));
        int arc = arcIndex(TrajectoryBuilder.build(points));
        r.check(arc >= 0, "corner has an arc (index %d)", arc);
        double maxCmdOnArc = run.cmds.stream().filter(c -> c.segment() == arc)
                .mapToDouble(c -> Math.hypot(c.speeds().vxMetersPerSecond, c.speeds().vyMetersPerSecond)).max().orElse(0);
        double maxSimOnArc = run.samples.stream().filter(s -> s.segment() == arc).mapToDouble(Sample::simSpeed).max().orElse(0);
        double entry = run.samples.stream().filter(s -> s.segment() == arc).mapToDouble(Sample::simSpeed).findFirst().orElse(0);
        r.check(maxCmdOnArc <= cap + 1e-9, "max commanded speed on the arc %.3f <= cap %.2f", maxCmdOnArc, cap);
        // ASSUMPTION: 0.15 m/s allowance for the 2-loop sensor/drive lag in sim.
        // KNOWN (oldPath-preserved, same LineSegment.isFinished + trapezoid at git HEAD): the line plans to
        // reach the corner speed AT its end but finishes maxPositionThreshold early, so the arc is entered
        // at about sqrt(cap^2 + 2 * maxAccel * maxPositionThreshold), and the arc then steps down to cap.
        r.known(maxSimOnArc <= cap + 0.15, "max true speed on the arc %.3f <= cap + 0.15 (overshoot %.3f m/s)",
                maxSimOnArc, maxSimOnArc - cap);
        // Bound derivation (deterministic harness: the follower sees its previous loop's command as
        // measured speed, and a sample shows the speed commanded one loop earlier). The highest true
        // speed on the arc is the line's command on its last cycle, made while the robot was still up to
        // one loop of travel (v * DT) beyond the threshold: v = sqrt(cap^2 + 2*a*(thr + v*DT)), i.e.
        // v^2 - 2*a*DT*v - (cap^2 + 2*a*thr) = 0. +0.01 m/s for the trapezoid's discrete steps.
        // (The first bound, sqrt(cap^2 + 2*a*thr) + 0.05 = 1.315, left out the v*DT travel and was set
        // from one real-time run, 1.254; real-time runs gave 1.254..1.322 because the sensor delay jumped
        // between 1 and 2 loops with wall-clock jitter.)
        double a = DEFAULT.maxAccel, dt = PathSimulationHarness.DT, thr = DEFAULT.maxPositionThreshold;
        double quirkEntry = a * dt + Math.sqrt(a * dt * a * dt + cap * cap + 2 * a * thr);
        r.check(maxSimOnArc <= quirkEntry + 0.01, "regression bound: max true speed on the arc %.3f <= "
                + "v from v^2 = cap^2 + 2*maxAccel*(maxPositionThreshold + v*DT): %.3f + 0.01", maxSimOnArc, quirkEntry);
        r.note("true speed at arc entry %.3f m/s (planned line end speed %.2f)", entry,
                TrajectoryBuilder.build(points).getSegments().get(arc - 1).getEndVelocity());
        r.key("arc cmd %.3f, arc true %.3f (overshoot %.3f), entry %.3f", maxCmdOnArc, maxSimOnArc, maxSimOnArc - cap, entry);
        r.finish();
    }

    @Test
    void longStraightAtMaxSpeed() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(8, 0, 0));
        Run run = runPath("long-straight-8m", points);
        Report r = new Report("long-straight-8m");
        standardChecks(r, run, DEFAULT, last(points));
        double peak = run.maxSimSpeed();
        double overshoot = run.samples.stream().mapToDouble(s -> s.sim().getX()).max().orElse(0) - 8;
        r.check(peak >= 0.9 * DEFAULT.maxSpeed, "peak true speed %.3f >= 90%% of maxSpeed (%.2f)", peak, 0.9 * DEFAULT.maxSpeed);
        r.check(overshoot < DEFAULT.finalPositionThreshold, "max overshoot past the end %.3f m < finalPositionThreshold %.2f",
                overshoot, DEFAULT.finalPositionThreshold);
        r.key("peak %.3f m/s, overshoot %.3f m", peak, overshoot);
        r.finish();
    }

    @Test
    void startHeading90DegOff() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(3, 0, 0));
        Run run = runPath("start-heading-90-off", pose(0, 0, 90), points, DEFAULT);
        Report r = new Report("start-heading-90-off");
        standardChecks(r, run, DEFAULT, last(points));
        double tTranslationDone = run.samples.stream().filter(s -> s.sim().getX() > 2.9).mapToDouble(Sample::time)
                .findFirst().orElse(Double.NaN);
        r.note("position reached at %.2f s, heading finished at %.2f s (max %.1f rad/s)",
                tTranslationDone, run.duration, DEFAULT.maxAngularSpeed);
        r.note("max |measured - commanded speed| one loop later %.3f m/s", run.maxSpeedRoundTripError()[0]);
        r.key("pos at %.2fs, done %.2fs, max omega %.2f", tTranslationDone, run.duration, run.maxCmdOmega());
        r.finish();
    }

    @Test
    void startOffsetFromFirstWaypoint() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(3, 0, 0), wp(3, 3, 0));
        Run run = runPath("start-offset-0.5m", pose(0, 0.5, 0), points, DEFAULT);
        Report r = new Report("start-offset-0.5m");
        double convergedFrom = run.samples.stream().filter(s -> s.sim().getX() >= 2.0).mapToDouble(Sample::time)
                .findFirst().orElse(0);
        standardChecks(r, run, DEFAULT, last(points), convergedFrom);
        double devAt1 = run.samples.stream().filter(s -> s.sim().getX() >= 1.0).mapToDouble(Sample::pathDeviation)
                .findFirst().orElse(Double.NaN);
        r.note("deviation at x=1.0: %.3f m; checked from x>=2.0 (t=%.2f s)", devAt1, convergedFrom);
        r.key("dev at x=1 %.3f", devAt1);
        r.finish();
    }

    @Test
    void chainedTwoPaths() throws Exception {
        List<Waypoint> a = List.of(wp(0, 0, 0), wp(2, 0, 0), wp(2, 2, 90));
        List<Waypoint> b = List.of(wp(2, 2, 90), wp(0, 2, 90), wp(0, 0, 180));
        PathSimulationHarness.teleport(a.get(0));
        Run run = new Run("chained-two-paths");
        FollowTrajectoryCommand first = run.follow(TrajectoryBuilder.build(a), a, null, null);
        FollowTrajectoryCommand second = run.follow(TrajectoryBuilder.build(b), b, null, null);
        run.execute(Commands.sequence(first, second), TIMEOUT, null);
        run.saveCsv();
        Report r = new Report("chained-two-paths");
        standardChecks(r, run, DEFAULT, last(b));
        // what happened at the hand-over
        int lastOfFirst = -1;
        for (int i = 0; i < run.cmds.size(); i++) {
            if (run.cmds.get(i).pathIndex() == 0) {
                lastOfFirst = i;
            }
        }
        boolean secondRan = lastOfFirst >= 0 && lastOfFirst + 1 < run.cmds.size();
        r.check(secondRan, "second path ran after the first");
        if (secondRan) {
            Cmd handA = run.cmds.get(lastOfFirst), handB = run.cmds.get(lastOfFirst + 1);
            r.note("hand-over: first path's last command at %.2f s, second path's first at %.2f s (%.0f loops gap)",
                    handA.time(), handB.time(), (handB.time() - handA.time()) / PathSimulationHarness.DT);
            Sample at = run.samples.stream().filter(s -> s.time() >= handB.time()).findFirst().orElse(null);
            if (at != null) {
                r.note("code pose at hand-over (%.3f, %.3f) vs first path end (2, 2): %.3f m (no pose reset)",
                        at.code().getX(), at.code().getY(), at.code().getTranslation().getDistance(new Translation2d(2, 2)));
            }
        }
        r.finish();
    }

    @Test
    void interruptMidPath() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(6, 0, 0));
        PathSimulationHarness.teleport(points.get(0));
        Run run = new Run("interrupt-mid-path");
        FollowTrajectoryCommand command = run.follow(TrajectoryBuilder.build(points), points, null, null);
        int cancelLoop = 60;
        run.samplesAfterEnd = 30;
        double[] speedAtCancel = new double[1];
        run.execute(command, TIMEOUT, (k, c) -> {
            if (k == cancelLoop) {
                speedAtCancel[0] = PathSimulationHarness.simSpeed();
                run.markHook();
                CommandScheduler.getInstance().cancel(c);
            }
        });
        run.saveCsv();
        Report r = new Report("interrupt-mid-path");
        r.check(run.ended && !run.timedOut, "command left the scheduler after cancel at %.2f s", cancelLoop * PathSimulationHarness.DT);
        r.check(run.nonFiniteCommands == 0, "non-finite commands: %d", run.nonFiniteCommands);
        ChassisSpeeds lastCmd = run.cmds.get(run.cmds.size() - 1).speeds();
        boolean zero = lastCmd.vxMetersPerSecond == 0 && lastCmd.vyMetersPerSecond == 0 && lastCmd.omegaRadiansPerSecond == 0;
        r.check(zero, "last speeds sent to the chassis are exactly zero (%s)", lastCmd);
        r.check(speedAtCancel[0] > 1.0, "robot was moving when cancelled: %.3f m/s", speedAtCancel[0]);
        // after cancel: samples after hookSample
        double speedAfter10 = Double.NaN;
        double maxAfter25 = 0;
        int base = Math.max(run.hookSample, 0);
        List<Sample> s = run.samples;
        if (base + 10 < s.size()) {
            speedAfter10 = s.get(base + 10).simSpeed();
        }
        // the default command takes over: the robot must stay still
        for (int i = base + 10; i < s.size(); i++) {
            maxAfter25 = Math.max(maxAfter25, s.get(i).simSpeed());
        }
        double coast = run.finalSim.getTranslation().getDistance(s.get(Math.min(base, s.size() - 1)).sim().getTranslation());
        r.check(!Double.isNaN(speedAfter10) && speedAfter10 < 0.05, "true speed 10 loops after cancel %.3f (< 0.05)", speedAfter10);
        r.check(maxAfter25 < 0.05, "stays stopped afterwards (max %.3f m/s)", maxAfter25);
        r.note("distance moved after cancel %.3f m, samples after cancel %d", coast, s.size() - 1 - base);
        r.key("v at cancel %.2f, last cmd zero %s, coast %.3f m", speedAtCancel[0], zero, coast);
        r.finish();
    }

    @Test
    void rerunSameCommandAndTrajectory() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(2, 0, 0), wp(2, 1.5, 90));
        DemaciaTrajectory trajectory = TrajectoryBuilder.build(points);
        Report r = new Report("rerun-same-command");
        List<Run> runs = new ArrayList<>();
        Run recorder = new Run("rerun");
        FollowTrajectoryCommand command = recorder.follow(trajectory, points, null, null);
        // the same command object, scheduled twice
        for (int i = 0; i < 2; i++) {
            PathSimulationHarness.teleport(points.get(0));
            recorder.execute(command, TIMEOUT, null);
            Run result = recorder.takeResult("rerun-" + (i + 1));
            result.saveCsv();
            runs.add(result);
        }
        // a new command sharing the same trajectory object
        PathSimulationHarness.teleport(points.get(0));
        Run third = new Run("rerun-3-new-command");
        third.execute(third.follow(trajectory, points, null, null), TIMEOUT, null);
        third.saveCsv();
        runs.add(third);
        for (int i = 0; i < runs.size(); i++) {
            Report sub = new Report("rerun-" + (i + 1));
            standardChecks(sub, runs.get(i), DEFAULT, last(points));
            String prefix = "run " + (i + 1);
            r.lines.addAll(sub.lines.stream().map(l -> prefix + l).toList());
            r.failed.addAll(sub.failed);
        }
        // the harness is deterministic and resets the robot on every teleport, so a rerun must be identical
        double t0 = runs.get(0).duration;
        double d0 = runs.get(0).finalSim.getTranslation().getDistance(last(points).getTranslation());
        for (int i = 1; i < runs.size(); i++) {
            Run run = runs.get(i);
            double d = run.finalSim.getTranslation().getDistance(last(points).getTranslation());
            r.check(Math.abs(run.duration - t0) < 1e-9 && Math.abs(d - d0) < 1e-12
                    && run.samples.size() == runs.get(0).samples.size(),
                    "run %d identical to the first: time %.2f vs %.2f s, final error %.6f vs %.6f m", i + 1,
                    run.duration, t0, d, d0);
        }
        r.key("times %.2f / %.2f / %.2f s", runs.get(0).duration, runs.get(1).duration, runs.get(2).duration);
        r.finish();
    }

    @Test
    void customPathConfig() throws Exception {
        PathConfig config = new PathConfig().withMaxSpeed(1.5).withMaxAccel(1.0);
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(3, 0, 0), wp(3, 2, 0));
        Run run = runPath("custom-config-1.5mps-1mps2", points.get(0), points, config);
        Report r = new Report("custom-config-1.5mps-1mps2");
        standardChecks(r, run, config, last(points));
        r.check(run.maxSimSpeed() <= config.maxSpeed + 0.02, "max true speed %.3f <= %.2f + 0.02", run.maxSimSpeed(),
                config.maxSpeed);
        r.check(run.maxSimAccel() <= config.maxAccel * 1.1, "max true accel (0.1 s window) %.3f <= 1.1 * %.2f",
                run.maxSimAccel(), config.maxAccel);
        r.key("true vmax %.3f, true accel %.2f", run.maxSimSpeed(), run.maxSimAccel());
        r.finish();
    }

    // ===================================================================== 3. robot-like disturbances

    /**
     * Drives a line while turning 180 deg. The follower asks for at most measured + maxAccel * DT
     * (0.06 m/s at default), so any measured-speed error bigger than that changes the motion: too low
     * and the robot crawls, too high and it accelerates past maxAccel / waits at stop points.
     * Measured speed is Chassis.getChassisSpeedsFieldRel (WPILib SwerveDriveKinematics.toChassisSpeeds, omega from the gyro).
     *
     * <p>Independent reference: the command sent one loop earlier. It goes out through the drive
     * command's inverse kinematics, the module optimizer (wheel flips) and the motor sim, and comes back
     * through Chassis's forward kinematics; the sim motors reach the commanded speed exactly. (Comparing
     * with the sim-world speed is not independent: both are WPILib kinematics of the same module states.)
     * The old DemaciaKinematics forward kinematics was off by up to ~0.2 m/s here.
     */
    @Test
    void measuredSpeedWhileRotating() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(4, 0, 180));
        Run run = runPath("measured-speed-while-rotating", points);
        Report r = new Report("measured-speed-while-rotating");
        standardChecks(r, run, DEFAULT, last(points));
        double allowed = DEFAULT.maxAccel * PathSimulationHarness.DT;
        double[] roundTrip = run.maxSpeedRoundTripError();
        r.check(roundTrip[0] <= allowed, "max |measured speed - speed commanded one loop earlier| %.3f m/s (at %.2f s) "
                + "<= maxAccel*DT %.3f", roundTrip[0], roundTrip[1], allowed);
        double minMid = run.samples.stream().filter(s -> s.sim().getX() > 0.5 && s.sim().getX() < 3.0)
                .mapToDouble(Sample::simSpeed).min().orElse(0);
        r.note("lowest true speed between x=0.5 and x=3.0: %.3f m/s; peak %.3f; max true accel %.2f",
                minMid, run.maxSimSpeed(), run.maxSimAccel());
        r.key("|meas-cmd| %.3f (allowed %.3f), min mid-line v %.3f", roundTrip[0], allowed, minMid);
        r.finish();
    }

    /**
     * PROBE, not asserted: the trapezoid accelerates from MEASURED speed (oldPath design). A drive that
     * only reaches part of the commanded speed (or reads low) settles where cmd - measured = maxAccel*DT.
     * Modelled by scaling the measured speed the follower sees.
     */
    @Test
    void probeUnderReadingMeasuredSpeed() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(8, 0, 0));
        Report r = new Report("probe-measured-speed-scale", true);
        for (double scale : new double[] { 0.98, 0.95 }) {
            Run run = runPath("probe-measured-x" + scale, points.get(0), points, DEFAULT, null,
                    s -> s.times(scale), null, TIMEOUT);
            double predicted = Math.min(DEFAULT.maxSpeed, DEFAULT.maxAccel * PathSimulationHarness.DT / (1 - scale));
            r.note("measured x%.2f: peak true speed %.3f m/s (fixed point maxAccel*DT/(1-scale) = %.2f), time %.2f s, "
                    + "finished %s", scale, run.maxSimSpeed(), predicted, run.duration, run.ended && !run.timedOut);
            r.key("x%.2f: peak %.2f m/s, %.2fs", scale, run.maxSimSpeed(), run.duration);
        }
        r.finish();
    }

    @Test
    void noisyMeasuredSpeed() throws Exception {
        Random random = new Random(5635);
        double sigma = 0.05; // m/s per axis
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(2, 0, 0).withMaxCornerSpeed(0), wp(2, 2, 0), wp(0, 2, 0));
        Run run = runPath("noisy-measured-speed", points.get(0), points, DEFAULT, null,
                s -> new ChassisSpeeds(s.vxMetersPerSecond + sigma * random.nextGaussian(),
                        s.vyMetersPerSecond + sigma * random.nextGaussian(), s.omegaRadiansPerSecond), null, TIMEOUT);
        Report r = new Report("noisy-measured-speed");
        standardChecks(r, run, DEFAULT, last(points));
        r.note("noise sigma %.2f m/s per axis on the measured field speeds", sigma);
        r.finish();
    }

    @Test
    void laggedMeasuredSpeed() throws Exception {
        int lag = 3; // extra loops on top of the sim's own 2
        ArrayDeque<ChassisSpeeds> queue = new ArrayDeque<>();
        double cap = 1.0;
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(5, 0, 0).withMaxCornerSpeed(cap), wp(5, 3, 0));
        Run run = runPath("lagged-measured-speed", points.get(0), points, DEFAULT, null, s -> {
            queue.addLast(s);
            return queue.size() > lag ? queue.removeFirst() : new ChassisSpeeds();
        }, null, TIMEOUT);
        Report r = new Report("lagged-measured-speed");
        standardChecks(r, run, DEFAULT, last(points));
        int arc = arcIndex(TrajectoryBuilder.build(points));
        double maxSimOnArc = run.samples.stream().filter(s -> s.segment() == arc).mapToDouble(Sample::simSpeed).max().orElse(0);
        double overshoot = run.samples.stream().mapToDouble(s -> s.sim().getY()).max().orElse(0) - 3;
        r.note("REPORT ONLY: +%d loops (%.0f ms) measured-speed lag: max true speed on 1 m/s arc %.3f, final overshoot %.3f m",
                lag, lag * 20.0, maxSimOnArc, overshoot);
        r.key("arc true %.3f (cap 1), end overshoot %.3f", maxSimOnArc, overshoot);
        r.finish();
    }

    private static Run poseJump(String name, Translation2d jump, java.util.function.Predicate<Pose2d> when) throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(4, 0, 0), wp(4, 3, 0));
        boolean[] done = new boolean[1];
        Run[] holder = new Run[1];
        PathSimulationHarness.teleport(points.get(0));
        Run run = new Run(name);
        holder[0] = run;
        FollowTrajectoryCommand command = run.follow(TrajectoryBuilder.build(points), points, null, null);
        run.execute(command, TIMEOUT, (k, c) -> {
            if (!done[0] && when.test(PathSimulationHarness.codePose())) {
                done[0] = true;
                holder[0].markHook();
                PathSimulationHarness.shiftBoth(jump);
            }
        });
        run.saveCsv();
        Report r = new Report(name);
        r.check(done[0], "pose jump %s happened", jump);
        double jumpTime = run.hookSample >= 0 ? run.samples.get(run.hookSample).time() : 0;
        standardChecks(r, run, DEFAULT, last(points), jumpTime + 1.0);
        double devBefore = run.maxPathDeviation(-1, jumpTime);
        double peakAfter = run.maxPathDeviation(jumpTime, jumpTime + 1.0);
        // biggest commanded vector change in the 0.2 s after the jump
        double vec = 0;
        for (int i = 1; i < run.cmds.size(); i++) {
            Cmd a = run.cmds.get(i - 1), b = run.cmds.get(i);
            if (b.time() >= jumpTime && b.time() <= jumpTime + 0.2) {
                vec = Math.max(vec, Math.hypot(b.speeds().vxMetersPerSecond - a.speeds().vxMetersPerSecond,
                        b.speeds().vyMetersPerSecond - a.speeds().vyMetersPerSecond) / PathSimulationHarness.DT);
            }
        }
        List<Integer> segmentsUsed = run.cmds.stream().map(Cmd::segment).distinct().toList();
        r.note("jump at %.2f s; deviation before %.3f, in the 1 s after %.3f m; max cmd vector change after jump %.1f m/s^2",
                jumpTime, devBefore, peakAfter, vec);
        r.note("segments used in order: %s of %d", segmentsUsed, TrajectoryBuilder.build(points).getSegments().size());
        r.key("jump@%.2fs, dev after %.3f, cmd dv %.1f m/s^2, segs %s", jumpTime, peakAfter, vec, segmentsUsed);
        r.finish();
        return run;
    }

    @Test
    void estimatorJumpLateral() throws Exception {
        poseJump("pose-jump-lateral-0.3m", new Translation2d(0, 0.3), p -> p.getX() >= 1.5);
    }

    @Test
    void estimatorJumpPastLineEnd() throws Exception {
        // forward jump that lands beyond the first line's end (inside the corner)
        poseJump("pose-jump-forward-1m", new Translation2d(1.0, 0), p -> p.getX() >= 2.5);
    }

    @Test
    void noisyEstimatedPose() throws Exception {
        Random random = new Random(1234);
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(2, -2, 90), wp(2, 0, 180), wp(1.5, -1, 270), wp(0, 0, 0));
        Run run = runPath("noisy-pose-2cm-0.5deg", points.get(0), points, DEFAULT,
                p -> new Pose2d(p.getX() + 0.02 * random.nextGaussian(), p.getY() + 0.02 * random.nextGaussian(),
                        p.getRotation().plus(Rotation2d.fromDegrees(0.5 * random.nextGaussian()))),
                null, null, TIMEOUT);
        // 99.7 % bounds of the injected noise: radial distance of a 2-D Gaussian (sigma 0.02 m) is
        // Rayleigh, 99.7 % below 0.02 * sqrt(-2 ln 0.003) = 0.068 m; heading 3 * 0.5 = 1.5 deg
        run.poseNoiseAllowanceM = 0.02 * Math.sqrt(-2 * Math.log(0.003));
        run.poseNoiseAllowanceDeg = 1.5;
        Report r = new Report("noisy-pose-2cm-0.5deg");
        standardChecks(r, run, DEFAULT, last(points));
        r.finish();
    }

    // ===================================================================== 2. NaN / Infinity hunt

    private static Run nanCase(String name, Pose2d start, List<Waypoint> points, double devFrom,
            boolean deviationIsKnownQuirk) throws Exception {
        Run run = runPath(name, start, points, DEFAULT);
        Report r = new Report(name);
        standardChecks(r, run, DEFAULT, last(points), devFrom, deviationIsKnownQuirk);
        if (deviationIsKnownQuirk) {
            DemaciaTrajectory t = TrajectoryBuilder.build(points);
            StringBuilder plan = new StringBuilder();
            for (BaseSegment s : t.getSegments()) {
                plan.append(String.format("[%s end %s v_end %.2f] ", s.getClass().getSimpleName(),
                        s.getEndPose().getTranslation(), s.getEndVelocity()));
            }
            r.note("planned segments: %s", plan);
        }
        r.finish();
        return run;
    }

    private static void nanCase(String name, Pose2d start, List<Waypoint> points) throws Exception {
        nanCase(name, start, points, -1, false);
    }

    @Test
    void nanDuplicateMiddleWaypoint() throws Exception {
        // duplicate waypoints are rejected at build time, so the robot never drives them
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class, () -> TrajectoryBuilder
                .build(List.of(wp(0, 0, 0), wp(2, 0, 0), wp(2, 0, 0), wp(2, 2, 90)), DEFAULT));
    }

    @Test
    void nanZeroLengthFirstLeg() throws Exception {
        org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class,
                () -> TrajectoryBuilder.build(List.of(wp(0, 0, 0), wp(0, 0, 0), wp(2, 0, 0)), DEFAULT));
    }

    @Test
    void nanRobotExactlyOnStopWaypoint() throws Exception {
        nanCase("nan-robot-on-stop-point", pose(0, 0, 0),
                List.of(wp(-2, 0, 0), wp(0, 0, 0).withMaxCornerSpeed(0), wp(0, 2, 90)));
    }

    @Test
    void nanRobotOnFinalWaypointHeadingOff() throws Exception {
        nanCase("nan-robot-on-final-point", pose(2, 0, 90), List.of(wp(0, 0, 0), wp(2, 0, 0)));
    }

    @Test
    void nanStationaryRobotOnArc() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(3, 0, 0), wp(3, 3, 0));
        ArcSegment arc = (ArcSegment) TrajectoryBuilder.build(points).getSegments().get(1);
        Translation2d middle = PathSimulationHarness.arcMiddle(arc);
        nanCase("nan-stationary-on-arc", new Pose2d(middle, Rotation2d.kZero), points);
    }

    @Test
    void nanRobotAtArcCenter() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(3, 0, 0), wp(3, 3, 0));
        ArcSegment arc = (ArcSegment) TrajectoryBuilder.build(points).getSegments().get(1);
        // the robot starts a whole radius off the path on purpose, so the deviation is checked from the
        // first time it is back within the limit (same idea as start-offset-0.5m)
        Pose2d start = new Pose2d(PathSimulationHarness.arcCenter(arc), Rotation2d.kZero);
        nanCaseFromBackOnPath("nan-robot-at-arc-center", start, points);
    }

    private static void nanCaseFromBackOnPath(String name, Pose2d start, List<Waypoint> points) throws Exception {
        Run run = runPath(name, start, points, DEFAULT);
        double back = run.samples.stream().filter(s -> s.pathDeviation() < PATH_DEVIATION_LIMIT).mapToDouble(Sample::time)
                .findFirst().orElse(Double.MAX_VALUE);
        Report r = new Report(name);
        standardChecks(r, run, DEFAULT, last(points), back);
        r.note("started %.3f m off the path, back within %.2f m at %.2f s", run.samples.get(0).pathDeviation(),
                PATH_DEVIATION_LIMIT, back);
        r.finish();
    }

    @Test
    void nanTinyCornerCap() throws Exception {
        nanCase("nan-tiny-cap-0.01", pose(0, 0, 0), List.of(wp(0, 0, 0), wp(3, 0, 0).withMaxCornerSpeed(0.01), wp(3, 3, 0)));
    }

    /**
     * PROBE, not asserted: one NaN measured speed sample and, separately, one NaN pose sample. This is
     * garbage input (the robot code never produced it in sim); it shows whether one bad sample is
     * absorbed or poisons the rest of the path.
     */
    @Test
    void probeSingleNaNInputs() throws Exception {
        List<Waypoint> points = List.of(wp(0, 0, 0), wp(3, 0, 0), wp(3, 2, 90));
        int[] calls = new int[1];
        Run speedRun = runPath("probe-nan-measured-speed", points.get(0), points, DEFAULT, null,
                s -> ++calls[0] == 30 ? new ChassisSpeeds(Double.NaN, Double.NaN, s.omegaRadiansPerSecond) : s, null, 10);
        int[] poseCalls = new int[1];
        Run poseRun = runPath("probe-nan-pose", points.get(0), points, DEFAULT,
                p -> ++poseCalls[0] == 30 ? new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromRadians(Double.NaN)) : p,
                null, null, 10);
        Report r = new Report("probe-single-nan-input", true);
        r.note("one NaN measured speed: %d non-finite commands, finished %s in %.2f s", speedRun.nonFiniteCommands,
                speedRun.ended && !speedRun.timedOut, speedRun.duration);
        r.note("one NaN pose: %d non-finite commands, finished %s in %.2f s (timeout 10 s)", poseRun.nonFiniteCommands,
                poseRun.ended && !poseRun.timedOut, poseRun.duration);
        r.key("NaN speed -> %d bad cmds; NaN pose -> %d bad cmds, %s", speedRun.nonFiniteCommands,
                poseRun.nonFiniteCommands, poseRun.timedOut ? "HANG (timeout)" : "finished");
        r.finish();
    }
}
