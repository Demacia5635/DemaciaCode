// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.kinematics.DemaciaKinematics;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.sensors.Cancoder;
import frc.demacia.utils.sensors.Pigeon;

public class Chassis extends SubsystemBase {

    private static Chassis instance;

    public static void initialize(ChassisConfig chassisConfig) {
        if (instance == null)
            instance = new Chassis(chassisConfig);
    }

    public static Chassis getInstance() {
        return instance;
    }

    private final ChassisConfig chassisConfig;

    public SwerveModule[] modules;
    public Pigeon gyro;
    
    private DemaciaKinematics demaciaKinematics;

    private ChassisSpeeds lastSpeedsFieldRel = new ChassisSpeeds();
    private double lastAccelTime = Timer.getFPGATimestamp();

    private double lastOmega = 0;
    private double lastOmegaTime = Timer.getFPGATimestamp();

    private Rotation2d[] lastAngles;

    private ChassisSpeeds lastCommandedSpeedsRobotRel = new ChassisSpeeds();
    private SwerveModuleState[] lastCommandedStates = new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };

    private Chassis(ChassisConfig chassisConfig) {
        setName(getName());

        this.chassisConfig = chassisConfig;
        modules = new SwerveModule[4];
        Translation2d[] modulePositions = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(chassisConfig.swerveModuleConfig[i]);
            modulePositions[i] = chassisConfig.swerveModuleConfig[i].position;
            if (modulePositions[i].getNorm() == 0) {
                Log.alert(getName() + " module location on the robot is 0");
            }

            if (chassisConfig.swerveModuleConfig[i].steerOffset == 0) {
                Log.alert(getName() + " steer offset is 0");
            }
        }
        lastAngles = new Rotation2d[4];
        for (int i = 0; i < lastAngles.length; i++) {
            lastAngles[i] = new Rotation2d(modules[i].getSteerAngle());
        }
        gyro = new Pigeon(chassisConfig.pigeonConfig);

        demaciaKinematics = new DemaciaKinematics(modulePositions);

        addLog();

        addDiagLogs();
    }

    public void addLog() {
        Log.putData("chassis/gyro angle", () -> getGyroAngle().getDegrees());

        SmartDashboard.putData("chassis/set coast",
                new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData("chassis/set brake",
                new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset moduls", new InstantCommand(()-> resetMudolse()).ignoringDisable(true));
    }

    public void checkElectronics() {
        for (SwerveModule module : modules) {
            module.checkElectronics();
        }
    }

    public void setNeutralMode(boolean isBrake) {
        for (SwerveModule module : modules) {
            module.setNeutralMode(isBrake);
        }
    }

    public ChassisConfig getConfig() {
        return chassisConfig;
    }

    public void resetMudolse(){
        for (int i = 0; i < modules.length; i++) {
            modules[i].resetModule();
        }
    }

    public Cancoder[] getCancoders() {
        Cancoder[] cancoders = new Cancoder[modules.length];
        for (int i = 0; i < modules.length; i++) {
            cancoders[i] = modules[i].getCancoder();
        }
        return cancoders;
    }

    public Rotation2d getGyroAngle() {
        return gyro.getGyroAngle();
    }

    public double getGyroAngularVelocity() {
        return gyro.getZVelocity();
    }

    public void setGyroYaw(Rotation2d angle) {
        if (angle != null) {
            gyro.setYaw(angle.getDegrees());
        }
    }

    public void stop() {
        lastCommandedSpeedsRobotRel = new ChassisSpeeds();
        for (int i = 0; i < lastCommandedStates.length; i++) {
            lastCommandedStates[i] = new SwerveModuleState(0, lastCommandedStates[i].angle);
        }
        for (SwerveModule i : modules) {
            i.stop();
        }
    }

    public void setSteerPower(double pow, int id) {
        modules[id].setSteerPower(pow);
    }

    public double getSteerVelocity(int id) {
        return modules[id].getSteerVel();
    }

    public double getSteerAcceleration(int id) {
        return modules[id].getSteerAccel();
    }

    public void setSteerPositions(double position) {
        setSteerPositions(new double[]{position, position, position, position});
    }

    public void setDrivePower(double pow, int id) {
        modules[id].setDrivePower(pow);
    }

    public void setDrivePower(double pow) {
        for (int i = 0; i < 4; i++)
            setDrivePower(pow, i);
    }

    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setSpeedsRobotRel(ChassisSpeeds robotRelSpeeds) {
        lastCommandedSpeedsRobotRel = robotRelSpeeds;

        boolean isStopped = Math.abs(robotRelSpeeds.vxMetersPerSecond) < 0.01 &&
            Math.abs(robotRelSpeeds.vyMetersPerSecond) < 0.01 &&
            Math.abs(robotRelSpeeds.omegaRadiansPerSecond) < 0.01;

        SwerveModuleState[] states = new SwerveModuleState[4];
        if (isStopped) {
            for (int i = 0; i < states.length; i++) {
                states[i] = new SwerveModuleState(0, lastAngles[i]);
            }
        } else {
            states = demaciaKinematics.toSwerveModuleStates(robotRelSpeeds);

            for (int i = 0; i < lastAngles.length; i++) {
                lastAngles[i] = states[i].angle;
            }
        }
        setModuleStates(states);

        if (RobotBase.isSimulation()) {
            gyro.getSimState().setRawYaw(Math.toDegrees(gyro.getCurrentYaw() + robotRelSpeeds.omegaRadiansPerSecond * 0.02));
        }
    }

    public void setSpeedsFieldRel(ChassisSpeeds fieldRelSpeeds) {
        setSpeedsRobotRel(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelSpeeds, getGyroAngle()));
    }

    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            lastCommandedStates[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
            modules[i].setState(states[i]);
        }
    }

    /** @return The last robot-relative speeds passed to {@link #setSpeedsRobotRel}. */
    public ChassisSpeeds getLastCommandedSpeedsRobotRel() {
        return lastCommandedSpeedsRobotRel;
    }

    /** @return A copy of the last module states passed to {@link #setModuleStates} (before optimization). */
    public SwerveModuleState[] getLastCommandedStates() {
        return lastCommandedStates.clone();
    }

    /** @return The swerve modules. */
    public SwerveModule[] getModules() {
        return modules;
    }

    public double getMaxDriveVelocity() {
        return chassisConfig.maxDriveVelocity;
    }

    public double getMaxRotationalVelocity() {
        return chassisConfig.maxRotationalVelocity;
    }

    public Translation2d getVelocityAsVector() {
        return new Translation2d(
                getChassisSpeedsFieldRel().vxMetersPerSecond,
                getChassisSpeedsFieldRel().vyMetersPerSecond);
    }

    public ChassisSpeeds getSpeedsRobotRel() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsFieldRel(), getGyroAngle());
    }

    /**
     * Returns linear acceleration [ax, ay] in m/s² (field-relative)
     * and angular acceleration [alpha] in rad/s², derived from velocity delta.
     */
    public double[] getAcceleration() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastAccelTime;

        ChassisSpeeds currentSpeedsFieldRel = getChassisSpeedsFieldRel();

        double ax = (currentSpeedsFieldRel.vxMetersPerSecond - lastSpeedsFieldRel.vxMetersPerSecond) / dt;
        double ay = (currentSpeedsFieldRel.vyMetersPerSecond - lastSpeedsFieldRel.vyMetersPerSecond) / dt;
        double aOmga = (currentSpeedsFieldRel.omegaRadiansPerSecond - lastSpeedsFieldRel.omegaRadiansPerSecond) / dt;

        lastSpeedsFieldRel = currentSpeedsFieldRel;
        lastAccelTime = now;

        return new double[]{ax, ay, aOmga};
    }

    /**
     * Returns angular acceleration (alpha) in rad/s² from the gyro.
     */
    public double getAngularAcceleration() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastOmegaTime;

        double currentOmega = getGyroAngularVelocity();
        double alpha = (currentOmega - lastOmega) / dt;

        lastOmega = currentOmega;
        lastOmegaTime = now;

        return alpha;
    }

    public ChassisSpeeds getChassisSpeedsRobotRel() {
        return demaciaKinematics.toChassisSpeeds(
                getModuleStates(),
                getGyroAngularVelocity());
    }

    public ChassisSpeeds getChassisSpeedsFieldRel() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                demaciaKinematics.toChassisSpeeds(getModuleStates(), getGyroAngularVelocity()),
                getGyroAngle());
    }

    public Translation2d getChassisSpeedsVector() {
        ChassisSpeeds s = getChassisSpeedsFieldRel();
        return new Translation2d(s.vxMetersPerSecond, s.vyMetersPerSecond);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] res = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = modules[i].getModulePosition();
        }
        return res;
    }

    public Translation2d[] getModuleLocations() {
        Translation2d[] res = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = chassisConfig.swerveModuleConfig[i].position;
        }
        return res;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] res = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = modules[i].getState();
        }
        return res;
    }

    @Override
    public void periodic() {
        if (RobotPose.getInstance() != null) {
            RobotPose.getInstance().periodic();
        }

        updateDiagnostics();
    }

    // ---- Diagnostics (diag/ logs) ----

    /**
     * Drive and odometry diagnostics (every log key starts with "diag/").
     *
     * <p>Used to find why the robot yaws while driving straight with no rotation command, and why
     * odometry drifts. Every loop {@link #updateDiagnostics()} reads the gyro, the commanded and
     * measured module states and the odometry, computes everything once and caches it. The log
     * suppliers only return the cached fields (no CAN reads inside suppliers).</p>
     *
     * <p>Conventions: WPILib (x forward, y left, CCW positive), meters, m/s, radians, rad/s.</p>
     *
     * <p>The main tool is a least-squares rigid-body fit of the measured module velocities. If all
     * wheels agree on one rigid motion, every module residual is ~0. A module that does not track
     * its command (steer not following, drive velocity off), or reports motion inconsistent with
     * the other modules, gets a large residual, and "omega fit minus gyro" shows rotation the
     * wheels do not report (scrub).</p>
     *
     * <p>Limits: all residuals are computed from encoder-reported module states, so they do NOT
     * detect a wrong constant steer offset, a wrong drive invert or a wheel diameter error (the
     * modules then agree with each other but not with reality). Those only show up as
     * "diag/fit/omega fit minus gyro" or the accumulated yaw difference, and need a physical
     * check (wheels aligned straight, push the robot a known distance).
     * "abs minus motor angle" only checks that the CANcoder and the steer motor still agree with
     * the boot-time seeding, not that the offset itself is right.</p>
     */
    private static final Log.LogLevel DIAG_LEVEL = Log.LogLevel.LOG_AND_NT_NOT_IN_COMP;
    private static final String[] DIAG_DEFAULT_NAMES = {"Front Left", "Front Right", "Back Left", "Back Right"};
    /** Below this commanded |v| (m/s) the chassis is not "translating". */
    private static final double MIN_TRANSLATION_SPEED = 0.2;
    /** Above this commanded |omega| (rad/s) the chassis is "rotating". */
    private static final double MAX_NO_ROT_OMEGA = 0.01;
    /** Largest dt (s) used for integration, so a long pause doesn't add a jump. */
    private static final double MAX_DT = 0.1;

    private double[] locX;
    private double[] locY;

    // gyro and commands
    private double gyroYaw, gyroRate;
    private double cmdVx, cmdVy, cmdOmega;

    // rigid-body fit
    private double[] measVx, measVy; // measured module velocity vectors (robot frame)
    private final double[] fit = new double[3];
    private final double[] looFit = new double[3];
    private double fitVx, fitVy, fitOmega, fitOmegaMinusGyro, maxResidual;

    // per module
    private double[] residual, looResidual, steerError, cmdAngle, cmdSpeed, measSpeed, speedError, measAngle;
    private double[] absMinusMotorAngle;
    private boolean[] flipped;
    private double[] rawDrivePos, couplingCorrection, correctedDrivePos, pathLength, lastCorrectedDrivePos;

    // driving straight
    private boolean translatingNoRot;
    // initialized to 0.0 (not NaN) so the log type is detected as double at registration
    private double yawRateWhileStraight = 0.0;
    private double accumulatedStraightYaw, accumulatedStraightWheelYaw;
    private double lastYaw = Double.NaN;
    private double lastTime = Double.NaN;

    // odometry
    private boolean poseAvailable;
    private double twistDx, twistDy, twistDtheta;
    private double odomX, odomY, odomTheta, estX, estY, estTheta;

    /**
     * Allocates the diagnostics arrays and registers every "diag/" log entry and the
     * "diag/reset accumulators" dashboard command. Called once at the end of the constructor.
     */
    private void addDiagLogs() {
        int n = modules.length;

        Translation2d[] locations = getModuleLocations();
        locX = new double[n];
        locY = new double[n];
        for (int i = 0; i < n; i++) {
            locX[i] = locations[i].getX();
            locY[i] = locations[i].getY();
        }

        measVx = new double[n];
        measVy = new double[n];
        residual = new double[n];
        looResidual = new double[n];
        steerError = new double[n];
        cmdAngle = new double[n];
        cmdSpeed = new double[n];
        measSpeed = new double[n];
        speedError = new double[n];
        measAngle = new double[n];
        absMinusMotorAngle = new double[n];
        flipped = new boolean[n];
        rawDrivePos = new double[n];
        couplingCorrection = new double[n];
        correctedDrivePos = new double[n];
        pathLength = new double[n];
        lastCorrectedDrivePos = new double[n];
        Arrays.fill(lastCorrectedDrivePos, Double.NaN);

        logDiagDouble("diag/gyro/yaw (rad)", () -> gyroYaw);
        logDiagDouble("diag/gyro/yaw rate (rad per s)", () -> gyroRate);

        logDiagDouble("diag/cmd/vx (m per s)", () -> cmdVx);
        logDiagDouble("diag/cmd/vy (m per s)", () -> cmdVy);
        logDiagDouble("diag/cmd/omega (rad per s)", () -> cmdOmega);

        logDiagDouble("diag/fit/vx (m per s)", () -> fitVx);
        logDiagDouble("diag/fit/vy (m per s)", () -> fitVy);
        logDiagDouble("diag/fit/omega (rad per s)", () -> fitOmega);
        logDiagDouble("diag/fit/omega fit minus gyro (rad per s)", () -> fitOmegaMinusGyro);
        logDiagDouble("diag/fit/max residual (m per s)", () -> maxResidual);

        for (int i = 0; i < n; i++) {
            final int m = i;
            String p = "diag/module/" + diagModuleName(i) + "/";
            logDiagDouble(p + "residual (m per s)", () -> residual[m]);
            // distance from the rigid motion fitted by the OTHER modules (isolates one bad module)
            logDiagDouble(p + "loo residual (m per s)", () -> looResidual[m]);
            // cmd angle - measured angle, folded to [-pi/2, pi/2]; only meaningful when cmd speed is not ~0
            logDiagDouble(p + "steer error (rad)", () -> steerError[m]);
            logDiagBoolean(p + "flipped", () -> flipped[m]);
            logDiagDouble(p + "cmd angle (rad)", () -> cmdAngle[m]);
            logDiagDouble(p + "measured angle (rad)", () -> measAngle[m]);
            // CANcoder absolute angle (steer offset applied as at boot) - steer motor angle, [-pi, pi];
            // drifts from 0 if the steer motor lost its seed (slip, brownout, resetModule)
            logDiagDouble(p + "abs minus motor angle (rad)", () -> absMinusMotorAngle[m]);
            logDiagDouble(p + "cmd speed (m per s)", () -> cmdSpeed[m]);
            // sign flipped when the module runs 180 deg from the command
            logDiagDouble(p + "measured speed (m per s)", () -> measSpeed[m]);
            logDiagDouble(p + "speed error (m per s)", () -> speedError[m]);
            logDiagDouble(p + "raw drive position (m)", () -> rawDrivePos[m]);
            logDiagDouble(p + "coupling correction (m)", () -> couplingCorrection[m]);
            logDiagDouble(p + "corrected drive position (m)", () -> correctedDrivePos[m]);
            logDiagDouble(p + "path length (m)", () -> pathLength[m]);
        }

        logDiagBoolean("diag/straight/translating no rot", () -> translatingNoRot);
        // NaN when not translating without rotation
        logDiagDouble("diag/straight/yaw rate while straight (rad per s)", () -> yawRateWhileStraight);
        logDiagDouble("diag/straight/accumulated yaw (rad)", () -> accumulatedStraightYaw);
        logDiagDouble("diag/straight/accumulated wheel yaw (rad)", () -> accumulatedStraightWheelYaw);

        logDiagBoolean("diag/odometry/available", () -> poseAvailable);
        logDiagDouble("diag/odometry/twist dx (m)", () -> twistDx);
        logDiagDouble("diag/odometry/twist dy (m)", () -> twistDy);
        logDiagDouble("diag/odometry/twist dtheta (rad)", () -> twistDtheta);
        logDiagDouble("diag/odometry/odometry x (m)", () -> odomX);
        logDiagDouble("diag/odometry/odometry y (m)", () -> odomY);
        logDiagDouble("diag/odometry/odometry theta (rad)", () -> odomTheta);
        logDiagDouble("diag/odometry/estimated x (m)", () -> estX);
        logDiagDouble("diag/odometry/estimated y (m)", () -> estY);
        logDiagDouble("diag/odometry/estimated theta (rad)", () -> estTheta);

        SmartDashboard.putData("diag/reset accumulators",
                new InstantCommand(this::resetDiagAccumulators).ignoringDisable(true));
    }

    /**
     * Reads all inputs once and recomputes every logged value. Call once per loop, at the end
     * of {@link #periodic()}.
     */
    private void updateDiagnostics() {
        int n = modules.length;
        double now = Timer.getFPGATimestamp();
        double dt = Double.isFinite(lastTime) ? Math.min(Math.max(now - lastTime, 0), MAX_DT) : 0;
        lastTime = now;

        // gyro (Pigeon: yaw and rate are already in rad and rad/s, CCW positive)
        gyroYaw = finiteOr(getGyroAngle().getRadians(), gyroYaw);
        gyroRate = finiteOr(getGyroAngularVelocity(), 0);

        ChassisSpeeds cmd = getLastCommandedSpeedsRobotRel();
        cmdVx = cmd != null ? cmd.vxMetersPerSecond : 0;
        cmdVy = cmd != null ? cmd.vyMetersPerSecond : 0;
        cmdOmega = cmd != null ? cmd.omegaRadiansPerSecond : 0;

        // measured module states and drive positions
        SwerveModuleState[] cmdStates = getLastCommandedStates();
        for (int i = 0; i < n; i++) {
            SwerveModuleState state = modules[i].getState();
            double speed = finiteOr(state.speedMetersPerSecond, 0);
            double angle = finiteOr(state.angle.getRadians(), 0);
            measAngle[i] = angle;
            measVx[i] = speed * Math.cos(angle);
            measVy[i] = speed * Math.sin(angle);
            absMinusMotorAngle[i] = wrapPi(modules[i].getAbsoluteSteerAngle() - modules[i].getSteerAngle());

            SwerveModuleState c = (cmdStates != null && i < cmdStates.length) ? cmdStates[i] : null;
            if (c != null) {
                cmdAngle[i] = c.angle.getRadians();
                cmdSpeed[i] = c.speedMetersPerSecond;
                double rawError = wrapPi(cmdAngle[i] - angle);
                flipped[i] = Math.abs(rawError) > Math.PI / 2;
                steerError[i] = foldHalfPi(rawError);
            } else {
                cmdAngle[i] = 0;
                cmdSpeed[i] = 0;
                flipped[i] = false;
                steerError[i] = 0;
            }
            measSpeed[i] = flipped[i] ? -speed : speed;
            speedError[i] = cmdSpeed[i] - measSpeed[i];

            rawDrivePos[i] = modules[i].getRawDrivePosition();
            couplingCorrection[i] = modules[i].getCouplingCorrection();
            double corrected = modules[i].getDrivePosition();
            correctedDrivePos[i] = corrected;
            if (Double.isFinite(corrected) && Double.isFinite(lastCorrectedDrivePos[i])) {
                pathLength[i] += Math.abs(corrected - lastCorrectedDrivePos[i]);
            }
            lastCorrectedDrivePos[i] = Double.isFinite(corrected) ? corrected : lastCorrectedDrivePos[i];
        }

        // rigid-body fit and residuals
        fitRigidBody(measVx, measVy, locX, locY, -1, fit);
        fitVx = fit[0];
        fitVy = fit[1];
        fitOmega = fit[2];
        fitOmegaMinusGyro = fitOmega - gyroRate;
        maxResidual = 0;
        for (int i = 0; i < n; i++) {
            residual[i] = Math.hypot(measVx[i] - (fitVx - fitOmega * locY[i]),
                    measVy[i] - (fitVy + fitOmega * locX[i]));
            maxResidual = Math.max(maxResidual, residual[i]);
        }
        for (int i = 0; i < n; i++) {
            fitRigidBody(measVx, measVy, locX, locY, i, looFit);
            looResidual[i] = Math.hypot(measVx[i] - (looFit[0] - looFit[2] * locY[i]),
                    measVy[i] - (looFit[1] + looFit[2] * locX[i]));
        }

        // uncommanded heading change while translating with no rotation command
        translatingNoRot = Math.abs(cmdOmega) < MAX_NO_ROT_OMEGA
                && Math.hypot(cmdVx, cmdVy) > MIN_TRANSLATION_SPEED;
        yawRateWhileStraight = translatingNoRot ? gyroRate : Double.NaN;
        if (translatingNoRot) {
            // yaw delta (not rate * dt), so gyro-rate noise does not integrate
            if (Double.isFinite(lastYaw)) {
                accumulatedStraightYaw += wrapPi(gyroYaw - lastYaw);
            }
            accumulatedStraightWheelYaw += fitOmega * dt;
        }
        lastYaw = gyroYaw;

        updateDiagOdometry();
    }

    private void updateDiagOdometry() {
        RobotPose robotPose = RobotPose.getInstance();
        poseAvailable = robotPose != null;
        if (!poseAvailable) {
            return;
        }
        Twist2d twist = robotPose.getLastOdometryTwist();
        if (twist != null) {
            twistDx = twist.dx;
            twistDy = twist.dy;
            twistDtheta = twist.dtheta;
        }
        Pose2d odom = robotPose.getOdometryPose();
        if (odom != null) {
            odomX = odom.getX();
            odomY = odom.getY();
            odomTheta = odom.getRotation().getRadians();
        }
        Pose2d est = robotPose.getEstimatedPose();
        if (est != null) {
            estX = est.getX();
            estY = est.getY();
            estTheta = est.getRotation().getRadians();
        }
    }

    /**
     * Zeroes the per-module path lengths and the accumulated straight-driving yaw. Bound to the
     * "diag/reset accumulators" dashboard button.
     */
    private void resetDiagAccumulators() {
        Arrays.fill(pathLength, 0);
        accumulatedStraightYaw = 0;
        accumulatedStraightWheelYaw = 0;
    }

    /**
     * Least-squares rigid-body fit of module velocities.
     *
     * <p>Model: module i at (x_i, y_i) moves at (vx - w*y_i, vy + w*x_i). With the locations
     * shifted to their centroid c (x'_i = x_i - cx, y'_i = y_i - cy) the normal equations
     * decouple: the centroid velocity is the mean module velocity, and
     * w = sum(x'_i*vy_i - y'_i*vx_i) / sum(x'_i^2 + y'_i^2). Then vx = Vcx + w*cy and
     * vy = Vcy - w*cx move it back to the robot origin. If all locations coincide, w = 0.</p>
     *
     * @param mvx Module velocity x components (m/s).
     * @param mvy Module velocity y components (m/s).
     * @param x   Module x locations (m).
     * @param y   Module y locations (m).
     * @param skip Index of a module to leave out (leave-one-out fit), or -1 to use all.
     * @param out Filled with {vx, vy, omega} (m/s, m/s, rad/s) at the robot origin.
     */
    private static void fitRigidBody(double[] mvx, double[] mvy, double[] x, double[] y, int skip, double[] out) {
        int count = 0;
        double cx = 0, cy = 0, meanVx = 0, meanVy = 0;
        for (int i = 0; i < mvx.length; i++) {
            if (i == skip) {
                continue;
            }
            count++;
            cx += x[i];
            cy += y[i];
            meanVx += mvx[i];
            meanVy += mvy[i];
        }
        if (count == 0) {
            out[0] = out[1] = out[2] = 0;
            return;
        }
        cx /= count;
        cy /= count;
        meanVx /= count;
        meanVy /= count;

        double num = 0, den = 0;
        for (int i = 0; i < mvx.length; i++) {
            if (i == skip) {
                continue;
            }
            double dx = x[i] - cx;
            double dy = y[i] - cy;
            num += dx * mvy[i] - dy * mvx[i];
            den += dx * dx + dy * dy;
        }
        double omega = den > 1e-12 ? num / den : 0;
        out[0] = meanVx + omega * cy;
        out[1] = meanVy - omega * cx;
        out[2] = omega;
    }

    /** Wraps an angle to [-pi, pi] (rad). NaN stays NaN. */
    private static double wrapPi(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Folds an angle error already wrapped to [-pi, pi] into [-pi/2, pi/2], treating a module
     * that runs 180 deg from the command (with reversed drive) as aligned.
     */
    private static double foldHalfPi(double wrappedError) {
        if (wrappedError > Math.PI / 2) {
            return wrappedError - Math.PI;
        }
        if (wrappedError < -Math.PI / 2) {
            return wrappedError + Math.PI;
        }
        return wrappedError;
    }

    private static double finiteOr(double value, double fallback) {
        return Double.isFinite(value) ? value : fallback;
    }

    private String diagModuleName(int i) {
        String name = modules[i].name;
        if (name != null && !name.isEmpty()) {
            return name;
        }
        return i < DIAG_DEFAULT_NAMES.length ? DIAG_DEFAULT_NAMES[i] : "module " + i;
    }

    @SuppressWarnings("unchecked")
    private static void logDiagDouble(String key, Supplier<Double> supplier) {
        Log.putData(key, new Supplier[] {supplier}, DIAG_LEVEL, "", true);
    }

    @SuppressWarnings("unchecked")
    private static void logDiagBoolean(String key, Supplier<Boolean> supplier) {
        Log.putData(key, new Supplier[] {supplier}, DIAG_LEVEL, "", true);
    }
}
