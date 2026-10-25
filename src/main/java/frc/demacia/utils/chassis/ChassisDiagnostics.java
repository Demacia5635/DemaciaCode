package frc.demacia.utils.chassis;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.utils.log.Log;

/**
 * Drive and odometry diagnostics for the swerve chassis.
 *
 * <p>Used to find why the robot yaws while driving straight with no rotation command, and why
 * odometry drifts. Every loop {@link #update()} reads the gyro, the commanded and measured
 * module states and the odometry, computes everything once and caches it. The log suppliers
 * only return the cached fields (no CAN reads inside suppliers).</p>
 *
 * <p>Conventions: WPILib (x forward, y left, CCW positive), meters, m/s, radians, rad/s.
 * Every key starts with "diag/".</p>
 *
 * <p>The main tool is a least-squares rigid-body fit of the measured module velocities. If all
 * wheels agree on one rigid motion, every module residual is ~0. A module whose real direction
 * or speed differs from the others (bad offset, steer not tracking, drive velocity off, wrong
 * invert) gets a large residual, and "omega fit minus gyro" shows rotation the wheels do not
 * report (scrub).</p>
 */
public class ChassisDiagnostics {

    private static final Log.LogLevel LEVEL = Log.LogLevel.LOG_AND_NT_NOT_IN_COMP;
    private static final String[] DEFAULT_NAMES = {"Front Left", "Front Right", "Back Left", "Back Right"};
    /** Below this commanded |v| (m/s) the chassis is not "translating". */
    private static final double MIN_TRANSLATION_SPEED = 0.2;
    /** Above this commanded |omega| (rad/s) the chassis is "rotating". */
    private static final double MAX_NO_ROT_OMEGA = 0.01;
    /** Largest dt (s) used for integration, so a long pause doesn't add a jump. */
    private static final double MAX_DT = 0.1;

    private final Chassis chassis;
    private final SwerveModule[] modules;
    private final int n;
    private final double[] locX;
    private final double[] locY;

    // gyro and commands
    private double gyroYaw, gyroRate;
    private double cmdVx, cmdVy, cmdOmega;

    // rigid-body fit
    private final double[] measVx, measVy; // measured module velocity vectors (robot frame)
    private final double[] fit = new double[3];
    private final double[] looFit = new double[3];
    private double fitVx, fitVy, fitOmega, fitOmegaMinusGyro, maxResidual;

    // per module
    private final double[] residual, looResidual, steerError, cmdAngle, cmdSpeed, measSpeed, speedError, measAngle;
    private final boolean[] flipped;
    private final double[] rawDrivePos, couplingCorrection, correctedDrivePos, pathLength, lastCorrectedDrivePos;

    // driving straight
    private boolean translatingNoRot;
    private double yawRateWhileStraight, accumulatedStraightYaw, accumulatedStraightWheelYaw;
    private double lastYaw = Double.NaN;
    private double lastTime = Double.NaN;

    // odometry
    private boolean poseAvailable;
    private double twistDx, twistDy, twistDtheta;
    private double odomX, odomY, odomTheta, estX, estY, estTheta;

    /**
     * Creates the diagnostics and registers every log entry and the reset command.
     *
     * @param chassis The chassis to diagnose.
     */
    public ChassisDiagnostics(Chassis chassis) {
        this.chassis = chassis;
        this.modules = chassis.getModules();
        this.n = modules.length;

        Translation2d[] locations = chassis.getModuleLocations();
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
        flipped = new boolean[n];
        rawDrivePos = new double[n];
        couplingCorrection = new double[n];
        correctedDrivePos = new double[n];
        pathLength = new double[n];
        lastCorrectedDrivePos = new double[n];
        java.util.Arrays.fill(lastCorrectedDrivePos, Double.NaN);

        addLog();
    }

    private void addLog() {
        logDouble("diag/gyro/yaw (rad)", () -> gyroYaw);
        logDouble("diag/gyro/yaw rate (rad per s)", () -> gyroRate);

        logDouble("diag/cmd/vx (m per s)", () -> cmdVx);
        logDouble("diag/cmd/vy (m per s)", () -> cmdVy);
        logDouble("diag/cmd/omega (rad per s)", () -> cmdOmega);

        logDouble("diag/fit/vx (m per s)", () -> fitVx);
        logDouble("diag/fit/vy (m per s)", () -> fitVy);
        logDouble("diag/fit/omega (rad per s)", () -> fitOmega);
        logDouble("diag/fit/omega fit minus gyro (rad per s)", () -> fitOmegaMinusGyro);
        logDouble("diag/fit/max residual (m per s)", () -> maxResidual);

        for (int i = 0; i < n; i++) {
            final int m = i;
            String p = "diag/module/" + moduleName(i) + "/";
            logDouble(p + "residual (m per s)", () -> residual[m]);
            // distance from the rigid motion fitted by the OTHER modules (isolates one bad module)
            logDouble(p + "loo residual (m per s)", () -> looResidual[m]);
            // cmd angle - measured angle, folded to [-pi/2, pi/2]; only meaningful when cmd speed is not ~0
            logDouble(p + "steer error (rad)", () -> steerError[m]);
            logBoolean(p + "flipped", () -> flipped[m]);
            logDouble(p + "cmd angle (rad)", () -> cmdAngle[m]);
            logDouble(p + "measured angle (rad)", () -> measAngle[m]);
            logDouble(p + "cmd speed (m per s)", () -> cmdSpeed[m]);
            // sign flipped when the module runs 180 deg from the command
            logDouble(p + "measured speed (m per s)", () -> measSpeed[m]);
            logDouble(p + "speed error (m per s)", () -> speedError[m]);
            logDouble(p + "raw drive position (m)", () -> rawDrivePos[m]);
            logDouble(p + "coupling correction (m)", () -> couplingCorrection[m]);
            logDouble(p + "corrected drive position (m)", () -> correctedDrivePos[m]);
            logDouble(p + "path length (m)", () -> pathLength[m]);
        }

        logBoolean("diag/straight/translating no rot", () -> translatingNoRot);
        logDouble("diag/straight/yaw rate while straight (rad per s)", () -> yawRateWhileStraight);
        logDouble("diag/straight/accumulated yaw (rad)", () -> accumulatedStraightYaw);
        logDouble("diag/straight/accumulated wheel yaw (rad)", () -> accumulatedStraightWheelYaw);

        logBoolean("diag/odometry/available", () -> poseAvailable);
        logDouble("diag/odometry/twist dx (m)", () -> twistDx);
        logDouble("diag/odometry/twist dy (m)", () -> twistDy);
        logDouble("diag/odometry/twist dtheta (rad)", () -> twistDtheta);
        logDouble("diag/odometry/odometry x (m)", () -> odomX);
        logDouble("diag/odometry/odometry y (m)", () -> odomY);
        logDouble("diag/odometry/odometry theta (rad)", () -> odomTheta);
        logDouble("diag/odometry/estimated x (m)", () -> estX);
        logDouble("diag/odometry/estimated y (m)", () -> estY);
        logDouble("diag/odometry/estimated theta (rad)", () -> estTheta);

        SmartDashboard.putData("diag/reset accumulators",
                new InstantCommand(this::resetAccumulators).ignoringDisable(true));
    }

    /**
     * Reads all inputs once and recomputes every logged value. Call once per loop, at the end
     * of {@code Chassis.periodic()}.
     */
    public void update() {
        double now = Timer.getFPGATimestamp();
        double dt = Double.isFinite(lastTime) ? Math.min(Math.max(now - lastTime, 0), MAX_DT) : 0;
        lastTime = now;

        // gyro (Pigeon: yaw and rate are already in rad and rad/s, CCW positive)
        gyroYaw = finiteOr(chassis.getGyroAngle().getRadians(), gyroYaw);
        gyroRate = finiteOr(chassis.getGyroAngularVelocity(), 0);

        ChassisSpeeds cmd = chassis.getLastCommandedSpeedsRobotRel();
        cmdVx = cmd != null ? cmd.vxMetersPerSecond : 0;
        cmdVy = cmd != null ? cmd.vyMetersPerSecond : 0;
        cmdOmega = cmd != null ? cmd.omegaRadiansPerSecond : 0;

        // measured module states and drive positions
        SwerveModuleState[] cmdStates = chassis.getLastCommandedStates();
        for (int i = 0; i < n; i++) {
            SwerveModuleState state = modules[i].getState();
            double speed = finiteOr(state.speedMetersPerSecond, 0);
            double angle = finiteOr(state.angle.getRadians(), 0);
            measAngle[i] = angle;
            measVx[i] = speed * Math.cos(angle);
            measVy[i] = speed * Math.sin(angle);

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
        yawRateWhileStraight = translatingNoRot ? gyroRate : 0;
        if (translatingNoRot) {
            // yaw delta (not rate * dt), so gyro-rate noise does not integrate
            if (Double.isFinite(lastYaw)) {
                accumulatedStraightYaw += wrapPi(gyroYaw - lastYaw);
            }
            accumulatedStraightWheelYaw += fitOmega * dt;
        }
        lastYaw = gyroYaw;

        updateOdometry();
    }

    private void updateOdometry() {
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
    public void resetAccumulators() {
        java.util.Arrays.fill(pathLength, 0);
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
    static void fitRigidBody(double[] mvx, double[] mvy, double[] x, double[] y, int skip, double[] out) {
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
    static double wrapPi(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Folds an angle error already wrapped to [-pi, pi] into [-pi/2, pi/2], treating a module
     * that runs 180 deg from the command (with reversed drive) as aligned.
     */
    static double foldHalfPi(double wrappedError) {
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

    private String moduleName(int i) {
        String name = modules[i].name;
        if (name != null && !name.isEmpty()) {
            return name;
        }
        return i < DEFAULT_NAMES.length ? DEFAULT_NAMES[i] : "module " + i;
    }

    @SuppressWarnings("unchecked")
    private static void logDouble(String key, Supplier<Double> supplier) {
        Log.putData(key, new Supplier[] {supplier}, LEVEL, "", true);
    }

    @SuppressWarnings("unchecked")
    private static void logBoolean(String key, Supplier<Boolean> supplier) {
        Log.putData(key, new Supplier[] {supplier}, LEVEL, "", true);
    }
}
