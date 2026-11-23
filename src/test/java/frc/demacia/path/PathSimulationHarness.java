package frc.demacia.path;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.UnaryOperator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.path.segments.ArcSegment;
import frc.demacia.path.segments.BaseSegment;
import frc.demacia.utils.Data;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.Log;
import frc.robot.RobotContainer;

/**
 * Shared full-robot simulation harness for {@link PathSimulationScenariosTest}.
 *
 * <p>Runs the real RobotContainer / Chassis / RobotPose / FollowTrajectoryCommand with the HAL and
 * the Phoenix sim. Deterministic: WPILib time is paused and stepped 20 ms per loop, and each loop
 * waits until the Phoenix sim has published this loop's writes (see {@link #syncPhoenix()}); every
 * teleport also resets the wheels, drive encoders and sim gyro, so a scenario gives byte-identical
 * CSVs whatever ran before it. Keeps a
 * "sim world" pose built only from what the simulated wheels did (WPILib kinematics), which is the
 * truth the checks use. The RobotContainer is created once per JVM and reused by every scenario
 * (also when {@link PathSimulationTest} created it first).
 *
 * <p>Between scenarios the robot is stopped and "teleported": the estimator is reset and the sim
 * world pose is set to the same pose. Nothing else holds an absolute position, so this is the same
 * as physically placing the robot there.
 */
final class PathSimulationHarness {

    static final double DT = 0.02;

    private static boolean started;
    private static Chassis chassis;
    private static SwerveDriveKinematics kinematics;
    /** Truth: integrated from the simulated wheel states. */
    private static Pose2d simWorld = Pose2d.kZero;
    private static double time;
    /** Count of WPILib "x and y components of Rotation2d are zero" errors (a zero vector got an angle). */
    private static int rotationZeroErrors;
    /**
     * Wheel steer angles of a freshly started robot code after the warm-up (about PI in this sim).
     * Every teleport puts the wheels back here: the first steer move of a path couples into the drive
     * encoders (about 2.5 cm jump of the estimate in the first loops), and its direction depends on
     * where the wheels start, so this keeps every run like a fresh start.
     */
    private static double[] initialSteerAngles;

    private PathSimulationHarness() {
    }

    /**
     * Starts the shared simulated robot once per JVM (HAL, DS enabled, one RobotContainer, warm-up) and
     * pauses WPILib time; every {@link #loop()} then steps it by exactly 20 ms. Safe to call again.
     */
    static synchronized void start() throws InterruptedException {
        if (started) {
            SimHooks.pauseTiming();
            return;
        }
        assertTrue(HAL.initialize(500, 0), "HAL failed to start");
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        if (RobotContainer.driveCommand == null) {
            new RobotContainer();
        }
        chassis = Chassis.getInstance();
        kinematics = new SwerveDriveKinematics(chassis.getModuleLocations());
        putLogLast();
        SimHooks.pauseTiming();

        // Count zero-vector angle errors (they become DriverStation error spam on the robot)
        MathShared base = MathSharedStore.getMathShared();
        MathSharedStore.setMathShared(new MathShared() {
            @Override
            public void reportError(String error, StackTraceElement[] stackTrace) {
                if (error.contains("Rotation2d")) {
                    rotationZeroErrors++;
                }
                base.reportError(error, stackTrace);
            }

            @Override
            public void reportUsage(MathUsageId id, int count) {
                base.reportUsage(id, count);
            }

            @Override
            public double getTimestamp() {
                return base.getTimestamp();
            }
        });

        for (int i = 0; i < 25; i++) {
            loop();
        }
        simWorld = RobotPose.getInstance().getEstimatedPose();
        initialSteerAngles = new double[chassis.modules.length];
        for (int i = 0; i < initialSteerAngles.length; i++) {
            initialSteerAngles[i] = chassis.modules[i].getSteerAngle();
        }
        System.out.println("harness: steer angles after warm-up " + java.util.Arrays.toString(initialSteerAngles)
                + ", Phoenix sync failures " + phoenixSyncFailures + " " + lastSyncStatus
                + ", subsystem order " + subsystemOrder());
        started = true;
    }

    /**
     * Puts Log after Chassis in the scheduler, as on the robot. The scheduler runs subsystems in the
     * order they were registered. On the robot Log is first used inside the Chassis constructor, so
     * Chassis (which runs RobotPose) reads the sensors first and Log.periodic refreshes them after it:
     * 2 loops of sensor delay. In a full gradle build a path unit test that logs (FollowTrajectoryCommandTest)
     * can run first and register Log before Chassis exists: 1 loop of delay, so the results changed with
     * the test class order (gradle runs the classes that failed last time first). Unregister and register
     * again moves Log to the end.
     */
    private static void putLogLast() {
        Subsystem log = (Subsystem) field(null, Log.class, "logManager");
        CommandScheduler.getInstance().unregisterSubsystem(log);
        CommandScheduler.getInstance().registerSubsystem(log);
    }

    /** Names of the registered subsystems in the order the scheduler runs them (for the log). */
    @SuppressWarnings("unchecked")
    private static List<String> subsystemOrder() {
        Map<Subsystem, Command> subsystems = (Map<Subsystem, Command>) field(CommandScheduler.getInstance(),
                CommandScheduler.class, "m_subsystems");
        List<String> names = new ArrayList<>();
        for (Subsystem s : subsystems.keySet()) {
            names.add(s.getName());
        }
        return names;
    }

    /**
     * Leaves the shared simulated robot stopped at (0, 0, 0) with the wheels reset, and lets WPILib time
     * run again for any later test class in the same JVM (Gradle runs all test classes in one JVM).
     */
    static void restore() throws InterruptedException {
        if (!started) {
            return;
        }
        teleport(Pose2d.kZero);
        SimHooks.resumeTiming();
    }

    /** Diagnostics: loops where a Phoenix wait did not return OK (signal not refreshed in time). */
    static int phoenixSyncFailures;
    static String lastSyncStatus = "";

    /**
     * Makes the loop deterministic. The Phoenix sim runs on its own thread in real time and publishes
     * each status signal at its update rate, so with a plain 20 ms sleep a value written this loop
     * (setRotorVelocity, addRotorPosition, the gyro yaw) reached the robot code 1 or 2 loops later
     * depending on wall-clock jitter. Here every registered signal first drops frames already received,
     * then waits for TWO fresh frames: the second is sampled after the first arrived, i.e. after every
     * write of this loop, so the next loop always reads exactly this loop's writes (one loop of sensor
     * delay, every time).
     */
    static void syncPhoenix() {
        for (String name : new String[] { "rioSignalsArray", "canivoreSignalsArray" }) {
            BaseStatusSignal[] signals = (BaseStatusSignal[]) field(null, Data.class, name);
            if (signals.length == 0) {
                continue;
            }
            // consume frames that already arrived (they may have been sampled before this loop's writes),
            // then two fresh frames: the second is sampled after the first arrived, after every write
            BaseStatusSignal.refreshAll(signals);
            for (int i = 0; i < 2; i++) {
                StatusCode code = BaseStatusSignal.waitForAll(0.5, signals);
                if (!code.isOK()) {
                    phoenixSyncFailures++;
                    lastSyncStatus = name + ": " + code;
                }
            }
        }
    }

    /** Steer position of each module at the end of the last loop (rad), for {@link #simulateSteerCoupling()}. */
    private static double[] lastSteerPositions;

    /**
     * Test-side physics the motor sim leaves out. On the real MK5n, turning the steer also rolls the
     * wheel, so the drive encoder changes by -steerVelToDriveVel * (steer change) and
     * SwerveModule.getDrivePosition adds it back. The motor sim only integrates the commanded drive
     * speed, so every steer move used to show up in odometry as extra distance (2.9 cm on the first
     * 53 deg steer of the baseline route, more in every turn). This adds the coupling to the drive
     * motor sim, so the code's compensation cancels it as on the robot. ASSUMPTION: the size and sign
     * come from the code's own compensation (SwerveModuleConfig.steerVelToDriveVel); only the robot can
     * confirm them. Returns true if any drive encoder was moved.
     */
    static boolean simulateSteerCoupling() {
        Chassis c = Chassis.getInstance();
        if (lastSteerPositions == null) {
            lastSteerPositions = new double[c.modules.length];
            for (int i = 0; i < c.modules.length; i++) {
                lastSteerPositions[i] = steerPositionNow(c.modules[i]);
            }
            return false;
        }
        boolean moved = false;
        for (int i = 0; i < c.modules.length; i++) {
            double now = steerPositionNow(c.modules[i]);
            double change = now - lastSteerPositions[i];
            lastSteerPositions[i] = now;
            if (change == 0) {
                continue;
            }
            Object config = field(c.modules[i], c.modules[i].getClass(), "config");
            double metersPerRad = (double) field(config, config.getClass(), "steerVelToDriveVel");
            Object drive = field(c.modules[i], c.modules[i].getClass(), "driveMotor");
            try {
                java.lang.reflect.Method toSim = drive.getClass().getDeclaredMethod("toSimValue", double.class);
                toSim.setAccessible(true);
                double rotor = (double) toSim.invoke(drive, -metersPerRad * change);
                com.ctre.phoenix6.hardware.TalonFX talon = (com.ctre.phoenix6.hardware.TalonFX) field(drive,
                        drive.getClass(), "motor");
                talon.getSimState().addRotorPosition(rotor);
            } catch (ReflectiveOperationException e) {
                throw new IllegalStateException(e);
            }
            moved = true;
        }
        return moved;
    }

    /** The steer position Phoenix reports right now (signals were just refreshed by syncPhoenix). */
    private static double steerPositionNow(Object module) {
        frc.demacia.utils.motors.BaseMotor steer = (frc.demacia.utils.motors.BaseMotor) field(module,
                module.getClass(), "steerMotor");
        return steer.getPositionSignal().getSignal().getValueAsDouble();
    }

    /** Heading of the simulated robot (rad). The harness turns the sim gyro; Chassis skips it while time is paused. */
    private static double simYawRad;

    /**
     * Turns the sim gyro by the rotation the wheels made this loop (the wheel states Log.periodic read in
     * this loop, the same speeds the motor sim used to move the drive encoders). Done between loops,
     * before {@link #syncPhoenix()}, so the next sensor refresh always reads the gyro together with the
     * wheel step it belongs to. Pigeon yaw and yaw rate are registered in Data, so syncPhoenix waits for them.
     */
    static void turnSimGyro() {
        double omega = kinematics.toChassisSpeeds(chassis.getModuleStates()).omegaRadiansPerSecond;
        simYawRad += omega * DT;
        chassis.gyro.getSimState().setRawYaw(Math.toDegrees(simYawRad));
        chassis.gyro.getSimState().setAngularVelocityZ(Math.toDegrees(omega));
    }

    /**
     * Everything that happens between two robot loops in this harness: the sim gyro turns, WPILib time
     * steps exactly 20 ms (it is paused otherwise), the Phoenix sim publishes this loop's writes, and
     * the steer-drive coupling is added (synced again so the next loop reads it).
     */
    static void betweenLoops() {
        turnSimGyro();
        SimHooks.stepTiming(DT);
        syncPhoenix();
        if (simulateSteerCoupling()) {
            syncPhoenix();
        }
    }

    /** One robot loop, then move the sim world pose by what the wheels did. */
    static void loop() throws InterruptedException {
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().run();
        betweenLoops();
        time += DT;
        ChassisSpeeds wheel = kinematics.toChassisSpeeds(chassis.getModuleStates());
        simWorld = simWorld.exp(new Twist2d(wheel.vxMetersPerSecond * DT, wheel.vyMetersPerSecond * DT,
                wheel.omegaRadiansPerSecond * DT));
    }

    static Pose2d simWorld() {
        return simWorld;
    }

    static Pose2d codePose() {
        return RobotPose.getInstance().getEstimatedPose();
    }

    /** True translation speed from the simulated wheels (WPILib kinematics). */
    static double simSpeed() {
        ChassisSpeeds s = kinematics.toChassisSpeeds(chassis.getModuleStates());
        return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
    }

    static double simOmega() {
        return kinematics.toChassisSpeeds(chassis.getModuleStates()).omegaRadiansPerSecond;
    }

    /** Translation speed the follower sees (Chassis.getChassisSpeedsFieldRel, WPILib SwerveDriveKinematics + gyro omega). */
    static double measuredSpeed() {
        ChassisSpeeds s = chassis.getChassisSpeedsFieldRel();
        return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
    }

    /** Runs loops (default command, zero joystick) until the robot has stopped. */
    static void settle() throws InterruptedException {
        int still = 0;
        for (int i = 0; i < 150 && still < 5; i++) {
            loop();
            still = simSpeed() < 0.005 && Math.abs(simOmega()) < 0.005 ? still + 1 : 0;
        }
    }

    /** Stops the robot, then puts the estimator and the sim world at the same pose. */
    static void teleport(Pose2d pose) throws InterruptedException {
        settle();
        resetMechanism();
        RobotPose.getInstance().resetPose(pose);
        simWorld = pose;
        // a few still loops so the sensor delay has no motion in it
        for (int i = 0; i < 3; i++) {
            loop();
        }
        RobotPose.getInstance().resetPose(pose);
        simWorld = pose;
    }

    /**
     * Puts back the state a scenario leaves in the shared robot code, so every scenario starts the same
     * whatever ran before it (also when run alone with --tests): wheel steer angles (fresh-start values) (and the drive
     * command's remembered zero-speed headings, which re-steer the wheels on every zero command), and the
     * simulated gyro yaw. Test-only reflection; nothing in production is changed.
     */
    static void resetMechanism() throws InterruptedException {
        Object wpilibKinematics = field(RobotContainer.driveCommand, RobotContainer.driveCommand.getClass(), "kinematics");
        try {
            SwerveDriveKinematics k = (SwerveDriveKinematics) wpilibKinematics.getClass().getMethod("kinematics")
                    .invoke(wpilibKinematics);
            Rotation2d[] headings = new Rotation2d[chassis.modules.length];
            for (int i = 0; i < headings.length; i++) {
                headings[i] = new Rotation2d(initialSteerAngles[i]);
            }
            k.resetHeadings(headings);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException(e);
        }
        chassis.setSteerPositions(initialSteerAngles.clone());
        for (var module : chassis.modules) {
            ((frc.demacia.utils.motors.MotorInterface) field(module, module.getClass(), "driveMotor"))
                    .setEncoderPosition(0);
        }
        simYawRad = 0; // the sim gyro reads 0 again from the next turnSimGyro()
        // read the new values back before any command runs; otherwise the default command's zero-speed
        // state uses the old cached steer angle and steers the wheel straight back
        syncPhoenix();
        Data.refreshAll();
        // the reset steer move is not a real wheel roll: start the steer-drive coupling from here
        lastSteerPositions = null;
        simulateSteerCoupling();
        for (int i = 0; i < 3; i++) {
            loop();
        }
    }

    /** Shifts both the estimator and the truth (odometry slipped, vision then put it right). */
    static void shiftBoth(Translation2d delta) {
        Pose2d code = codePose();
        RobotPose.getInstance().resetPose(new Pose2d(code.getTranslation().plus(delta), code.getRotation()));
        simWorld = new Pose2d(simWorld.getTranslation().plus(delta), simWorld.getRotation());
    }

    // ------------------------------------------------------------------ reflection helpers (read only)

    static Object field(Object owner, Class<?> cls, String name) {
        try {
            Field f = cls.getDeclaredField(name);
            f.setAccessible(true);
            return f.get(owner);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("reflection failed for " + name, e);
        }
    }

    static int segmentIndex(FollowTrajectoryCommand command) {
        PathFollower follower = (PathFollower) field(command, FollowTrajectoryCommand.class, "follower");
        return (int) field(follower, PathFollower.class, "currentSegmentIndex");
    }

    static Translation2d segmentStart(BaseSegment segment) {
        return (Translation2d) field(segment, BaseSegment.class, "startTranslation");
    }

    static Translation2d arcCenter(ArcSegment arc) {
        return (Translation2d) field(arc, ArcSegment.class, "center");
    }

    /** Point in the middle of an arc. */
    static Translation2d arcMiddle(ArcSegment arc) {
        Translation2d c = arcCenter(arc);
        double start = (double) field(arc, ArcSegment.class, "startAngle");
        double sweep = (double) field(arc, ArcSegment.class, "sweep");
        boolean left = (boolean) field(arc, ArcSegment.class, "isLeft");
        double a = start + (left ? 1 : -1) * sweep / 2;
        return c.plus(new Translation2d(arc.getRadius() * Math.cos(a), arc.getRadius() * Math.sin(a)));
    }

    /** Distance from p to the built segment geometry (line piece or arc). */
    static double distanceToSegment(BaseSegment segment, Translation2d p) {
        Translation2d a = segmentStart(segment);
        Translation2d b = segment.getEndPose().getTranslation();
        if (segment instanceof ArcSegment arc) {
            Translation2d c = arcCenter(arc);
            double start = (double) field(arc, ArcSegment.class, "startAngle");
            double sweep = (double) field(arc, ArcSegment.class, "sweep");
            boolean left = (boolean) field(arc, ArcSegment.class, "isLeft");
            Translation2d cp = p.minus(c);
            if (cp.getNorm() < 1e-9) {
                return arc.getRadius();
            }
            double rel = (left ? 1 : -1) * (Math.atan2(cp.getY(), cp.getX()) - start);
            rel = ((rel % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
            if (rel <= sweep) {
                return Math.abs(cp.getNorm() - arc.getRadius());
            }
            return Math.min(p.getDistance(a), p.getDistance(b));
        }
        return distanceToPiece(a, b, p);
    }

    static double distanceToPiece(Translation2d a, Translation2d b, Translation2d p) {
        Translation2d ab = b.minus(a);
        double len2 = ab.getX() * ab.getX() + ab.getY() * ab.getY();
        if (len2 < 1e-12) {
            return p.getDistance(a);
        }
        Translation2d ap = p.minus(a);
        double t = Math.max(0, Math.min(1, (ap.getX() * ab.getX() + ap.getY() * ab.getY()) / len2));
        return p.getDistance(a.plus(ab.times(t)));
    }

    static double distanceToTrajectories(List<DemaciaTrajectory> trajectories, Translation2d p) {
        double best = Double.MAX_VALUE;
        for (DemaciaTrajectory t : trajectories) {
            for (BaseSegment s : t.getSegments()) {
                best = Math.min(best, distanceToSegment(s, p));
            }
        }
        return best;
    }

    static double distanceToPolyline(List<? extends Pose2d> points, Translation2d p) {
        double best = Double.MAX_VALUE;
        for (int i = 0; i + 1 < points.size(); i++) {
            best = Math.min(best, distanceToPiece(points.get(i).getTranslation(), points.get(i + 1).getTranslation(), p));
        }
        return best;
    }

    // ------------------------------------------------------------------ one run

    record Sample(double time, Pose2d sim, Pose2d code, ChassisSpeeds lastCmd, double simSpeed,
            double measuredSpeed, int pathIndex, int segment, double pathDeviation, double polylineDeviation) {
    }

    /** One call of the consumer. segment = follower segment index before calculate(). */
    record Cmd(double time, ChassisSpeeds speeds, double measuredSeen, int pathIndex, int segment) {
    }

    /** Collects everything for one scenario run. */
    static final class Run {
        final String name;
        final List<DemaciaTrajectory> trajectories = new ArrayList<>();
        final List<List<? extends Pose2d>> waypointLists = new ArrayList<>();
        final List<Sample> samples = new ArrayList<>();
        final List<Cmd> cmds = new ArrayList<>();
        int nonFiniteCommands;
        boolean ended;
        boolean timedOut;
        double duration;
        Pose2d finalSim;
        Pose2d finalCode;
        int rotationZeroErrors;
        /** Loops where the Phoenix signals did not refresh in time (would break determinism). */
        int syncFailures;
        /** Index (into samples) of the last sample before the scenario hook first acted, -1 if never. */
        int hookSample = -1;
        /** Samples still recorded after the command ended (before settling), e.g. to watch it stop. */
        int samplesAfterEnd = 0;

        // what the active command saw in this cycle
        private double measuredSeen;
        private int seenPath;
        private int seenSegment;
        private ChassisSpeeds lastCmd = new ChassisSpeeds();
        /** The pose the follower was given on its last cycle (after any test filter such as noise). */
        Pose2d lastSeenPose;
        /** 3 sigma of injected pose noise, metres / degrees (0 = no noise). */
        double poseNoiseAllowanceM;
        double poseNoiseAllowanceDeg;
        private int activePath;
        private FollowTrajectoryCommand activeCommand;
        private double startTime;

        Run(String name) {
            this.name = name;
        }

        /**
         * A FollowTrajectoryCommand wired like RobotContainer's, with recording wrappers.
         * poseFilter/speedFilter change what the follower sees (noise, lag, NaN); null = unchanged.
         * A non-finite command is recorded, then replaced by zero before the drive so the shared
         * simulated motors are not poisoned for later scenarios (the robot code would forward it).
         */
        FollowTrajectoryCommand follow(DemaciaTrajectory trajectory, List<? extends Pose2d> waypoints,
                UnaryOperator<Pose2d> poseFilter, UnaryOperator<ChassisSpeeds> speedFilter) {
            final int pathIndex = trajectories.size();
            trajectories.add(trajectory);
            waypointLists.add(waypoints);
            FollowTrajectoryCommand[] self = new FollowTrajectoryCommand[1];
            self[0] = new FollowTrajectoryCommand(trajectory,
                    () -> {
                        // called first in execute(): note which segment this cycle uses
                        activePath = pathIndex;
                        activeCommand = self[0];
                        seenPath = pathIndex;
                        seenSegment = segmentIndex(self[0]);
                        Pose2d pose = codePose();
                        lastSeenPose = poseFilter == null ? pose : poseFilter.apply(pose);
                        return lastSeenPose;
                    },
                    () -> {
                        ChassisSpeeds s = chassis.getChassisSpeedsFieldRel();
                        if (speedFilter != null) {
                            s = speedFilter.apply(s);
                        }
                        measuredSeen = Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
                        return s;
                    },
                    speeds -> {
                        boolean finite = Double.isFinite(speeds.vxMetersPerSecond)
                                && Double.isFinite(speeds.vyMetersPerSecond)
                                && Double.isFinite(speeds.omegaRadiansPerSecond);
                        cmds.add(new Cmd(time - startTime, speeds, measuredSeen, seenPath, seenSegment));
                        lastCmd = speeds;
                        if (!finite) {
                            nonFiniteCommands++;
                            speeds = new ChassisSpeeds();
                        }
                        RobotContainer.driveCommand.setVelFieldRel(speeds);
                    },
                    chassis);
            return self[0];
        }

        /**
         * Schedules the command and runs loops until it is no longer scheduled, or the timeout
         * (then it is cancelled and marked timedOut: the follower itself has no timeout).
         * hook(loopIndex, command) runs before every loop.
         */
        void execute(Command command, double timeoutSeconds, BiConsumer<Integer, Command> hook)
                throws InterruptedException {
            int errorsBefore = rotationZeroErrors();
            int syncBefore = phoenixSyncFailures;
            startTime = time;
            CommandScheduler.getInstance().schedule(command);
            int k = 0;
            while (true) {
                if (hook != null) {
                    hook.accept(k, command);
                }
                loop();
                k++;
                addSample();
                if (!CommandScheduler.getInstance().isScheduled(command)) {
                    ended = true;
                    break;
                }
                if (time - startTime >= timeoutSeconds) {
                    timedOut = true;
                    CommandScheduler.getInstance().cancel(command);
                    break;
                }
            }
            duration = time - startTime;
            for (int i = 0; i < samplesAfterEnd; i++) {
                loop();
                addSample();
            }
            settle();
            finalSim = simWorld;
            finalCode = codePose();
            rotationZeroErrors += rotationZeroErrors() - errorsBefore;
            syncFailures += phoenixSyncFailures - syncBefore;
        }

        /** Moves the results of the last execute() into a new Run, so the same command can run again. */
        Run takeResult(String resultName) {
            Run copy = new Run(resultName);
            copy.trajectories.addAll(trajectories);
            copy.waypointLists.addAll(waypointLists);
            copy.samples.addAll(samples);
            copy.cmds.addAll(cmds);
            copy.nonFiniteCommands = nonFiniteCommands;
            copy.ended = ended;
            copy.timedOut = timedOut;
            copy.duration = duration;
            copy.finalSim = finalSim;
            copy.finalCode = finalCode;
            copy.rotationZeroErrors = rotationZeroErrors;
            copy.hookSample = hookSample;
            samples.clear();
            cmds.clear();
            nonFiniteCommands = 0;
            ended = false;
            timedOut = false;
            rotationZeroErrors = 0;
            hookSample = -1;
            lastCmd = new ChassisSpeeds();
            return copy;
        }

        void markHook() {
            if (hookSample < 0) {
                hookSample = samples.size() - 1;
            }
        }

        private void addSample() {
            int segment = activeCommand == null ? -1 : segmentIndex(activeCommand);
            Translation2d p = simWorld.getTranslation();
            double polyline = Double.MAX_VALUE;
            for (List<? extends Pose2d> w : waypointLists) {
                polyline = Math.min(polyline, distanceToPolyline(w, p));
            }
            samples.add(new Sample(time - startTime, simWorld, codePose(), lastCmd, simSpeed(), measuredSpeed(),
                    activePath, segment, distanceToTrajectories(trajectories, p), polyline));
        }

        // -------------------------------------------------------------- metrics

        double maxPathDeviation(double fromTime, double toTime) {
            double m = 0;
            for (Sample s : samples) {
                if (s.time() >= fromTime && s.time() <= toTime) {
                    m = Math.max(m, s.pathDeviation());
                }
            }
            return m;
        }

        double maxPathDeviation() {
            return maxPathDeviation(-1, Double.MAX_VALUE);
        }

        double maxPolylineDeviation() {
            return samples.stream().mapToDouble(Sample::polylineDeviation).max().orElse(0);
        }

        double maxCmdSpeed() {
            return cmds.stream().mapToDouble(c -> Math.hypot(c.speeds().vxMetersPerSecond, c.speeds().vyMetersPerSecond))
                    .filter(Double::isFinite).max().orElse(0);
        }

        double maxCmdOmega() {
            return cmds.stream().mapToDouble(c -> Math.abs(c.speeds().omegaRadiansPerSecond))
                    .filter(Double::isFinite).max().orElse(0);
        }

        double maxSimSpeed() {
            return samples.stream().mapToDouble(Sample::simSpeed).max().orElse(0);
        }

        /** {max speed increase, max speed decrease, max vector change} of the command per cycle, / DT. */
        double[] cmdAccel() {
            double up = 0, down = 0, vec = 0;
            for (int i = 1; i < cmds.size(); i++) {
                ChassisSpeeds a = cmds.get(i - 1).speeds(), b = cmds.get(i).speeds();
                double va = Math.hypot(a.vxMetersPerSecond, a.vyMetersPerSecond);
                double vb = Math.hypot(b.vxMetersPerSecond, b.vyMetersPerSecond);
                if (!Double.isFinite(va) || !Double.isFinite(vb)) {
                    continue;
                }
                up = Math.max(up, (vb - va) / DT);
                down = Math.max(down, (va - vb) / DT);
                vec = Math.max(vec, Math.hypot(b.vxMetersPerSecond - a.vxMetersPerSecond,
                        b.vyMetersPerSecond - a.vyMetersPerSecond) / DT);
            }
            return new double[] { up, down, vec };
        }

        /** Largest true speed increase over a 0.1 s window, / 0.1 s (what the robot really did). */
        double maxSimAccel() {
            double m = 0;
            for (int i = 5; i < samples.size(); i++) {
                m = Math.max(m, (samples.get(i).simSpeed() - samples.get(i - 5).simSpeed()) / (5 * DT));
            }
            return m;
        }

        /** Number of commands asking for more than measured + maxAccel * DT (the trapezoid's own rule). */
        int accelRuleViolations(double maxAccel) {
            int n = 0;
            for (Cmd c : cmds) {
                double v = Math.hypot(c.speeds().vxMetersPerSecond, c.speeds().vyMetersPerSecond);
                if (Double.isFinite(v) && v > c.measuredSeen() + maxAccel * DT + 1e-9) {
                    n++;
                }
            }
            return n;
        }

        /** Max distance code pose vs sim pose 2 loops before (sensor delay, as in PathSimulationTest). */
        double maxEstimatorError() {
            double m = 0;
            for (int i = 2; i < samples.size(); i++) {
                m = Math.max(m, samples.get(i).code().getTranslation().getDistance(samples.get(i - 2).sim().getTranslation()));
            }
            return m;
        }

        /**
         * Round trip of the speed measurement: each command goes through the drive command's inverse
         * kinematics, the module optimizer and the motor sim, and comes back one loop later as the
         * measured speed the follower sees (Chassis.getChassisSpeedsFieldRel, forward kinematics + gyro).
         * In this sim the motors reach the commanded speed exactly, so any difference is an error in the
         * measurement path. Returns {max |measured - commanded speed|, at time}. Pairs in the same loop
         * (the zero sent by end()) are skipped.
         */
        double[] maxSpeedRoundTripError() {
            double max = 0, at = 0;
            for (int i = 0; i + 1 < cmds.size(); i++) {
                Cmd sent = cmds.get(i), next = cmds.get(i + 1);
                if (sent.pathIndex() != next.pathIndex() || next.time() - sent.time() < DT / 2) {
                    continue;
                }
                double commanded = Math.hypot(sent.speeds().vxMetersPerSecond, sent.speeds().vyMetersPerSecond);
                double error = Math.abs(next.measuredSeen() - commanded);
                if (Double.isFinite(error) && error > max) {
                    max = error;
                    at = next.time();
                }
            }
            return new double[] { max, at };
        }

        static double headingErrorDeg(Pose2d a, Pose2d b) {
            return Math.abs(a.getRotation().minus(b.getRotation()).getDegrees());
        }

        void saveCsv() throws IOException {
            Path file = Path.of("build", "path-sim-scenario-" + name + ".csv");
            Files.createDirectories(file.getParent());
            try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
                out.println("time,sim_x,sim_y,sim_deg,code_x,code_y,code_deg,cmd_vx,cmd_vy,cmd_omega,"
                        + "sim_speed,measured_speed,path,segment,path_dev,polyline_dev");
                for (Sample s : samples) {
                    out.printf("%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.5f,%.5f,%.5f,%.4f,%.4f,%d,%d,%.4f,%.4f%n",
                            s.time(), s.sim().getX(), s.sim().getY(), s.sim().getRotation().getDegrees(),
                            s.code().getX(), s.code().getY(), s.code().getRotation().getDegrees(),
                            s.lastCmd().vxMetersPerSecond, s.lastCmd().vyMetersPerSecond,
                            s.lastCmd().omegaRadiansPerSecond, s.simSpeed(), s.measuredSpeed(), s.pathIndex(),
                            s.segment(), s.pathDeviation(), s.polylineDeviation());
                }
            }
        }
    }

    static int rotationZeroErrors() {
        return rotationZeroErrors;
    }
}
