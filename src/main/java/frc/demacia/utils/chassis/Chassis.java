// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator; // ← חדש
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.demacia.kinematics.DemaciaKinematics;
import frc.demacia.odometry.DemaciaOdometry;
import frc.demacia.utils.dashboard.ElasticGenerator;
import frc.demacia.utils.sensors.Cancoder;
import frc.demacia.utils.sensors.Pigeon;
import frc.robot.RobotCommon;

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
    private SwerveDriveKinematics wpilibKinematics;

    // ── Pose Estimator (WPILib) ──────────────────────────────────────────────
    private SwerveDrivePoseEstimator poseEstimator;
    // ────────────────────────────────────────────────────────────────────────

    private Field2d field;
    private Field2d fieldTesting;
    private Field2d fieldOdmetry;

    private StatusSignal<Angle> gyroYawStatus;
    private StatusSignal<AngularVelocity> gyroAngularVelocityStatus;

    private Rotation2d lastGyroYaw;
    private double lastGyroAngularVelocity;

    private final PIDController xController = new PIDController(0.2, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.2, 0.0, 0.0);
    private final PIDController headingController = new PIDController(0.03, 0.0, 0) {
        {
            enableContinuousInput(-Math.PI, Math.PI);
        }
    };
    private int index = 0;

    private boolean isRotateToHub = false;

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private double lastAccelTime = Timer.getFPGATimestamp();

    private double lastOmega = 0;
    private double lastOmegaTime = Timer.getFPGATimestamp();
    private Translation2d[] modulePositions;

    private Chassis(ChassisConfig chassisConfig) {
        setName(getName());

        this.chassisConfig = chassisConfig;
        fieldOdmetry = new Field2d();
        modules = new SwerveModule[4];
        Translation2d[] modulePositions = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(chassisConfig.swerveModuleConfig[i]);
            modulePositions[i] = chassisConfig.swerveModuleConfig[i].position;
        }
        this.modulePositions = modulePositions;
        gyro = new Pigeon(chassisConfig.pigeonConfig);

        addStatus();
        demaciaKinematics = new DemaciaKinematics(modulePositions);
        wpilibKinematics = new SwerveDriveKinematics(modulePositions);

        poseEstimator = new SwerveDrivePoseEstimator(
                wpilibKinematics,
                getGyroAngle(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.03, 0.03, 0.01),
                VecBuilder.fill(0.9, 0.9, 0.9));

        field = new Field2d();
        fieldTesting = new Field2d();

        SmartDashboard.putData("chassis/field odometry", fieldOdmetry);
        SmartDashboard.putData("chassis/reset gyro",
                new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset gyro 180",
                new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
        SmartDashboard.putData("chassis/field", field);
        SmartDashboard.putData("chassis/fieldTesting", fieldTesting);
        SmartDashboard.putData("chassis/set coast",
                new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData("chassis/set brake",
                new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset odmetry",
                new InstantCommand(() -> DemaciaOdometry.getOdometryInstance(modulePositions)
                        .resetPose(getPose())).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset moduls", new InstantCommand(()-> resetMudolse()).ignoringDisable(true));

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        Cancoder[] cancoders = new Cancoder[4];
        for (int i = 0; i < 4; i++) {
            cancoders[i] = modules[i].cancoder;
        }
        ElasticGenerator.getInstance().registerChassisGyro(gyro);
        ElasticGenerator.getInstance().registerChassisCancoders(cancoders);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds,
                                     edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3,
                                             edu.wpi.first.math.numbers.N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
    }

    public void resetMudolse(){
        for (int i = 0; i < modules.length; i++) {
            modules[i].steerMotor.setEncoderPosition(0);
        }
    }

    /**
     * Returns linear acceleration [ax, ay] in m/s² (field-relative)
     * and angular acceleration [alpha] in rad/s², derived from velocity delta.
     */
    public double[] getAcceleration() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastAccelTime;

        ChassisSpeeds current = getChassisSpeedsFieldRel();

        double ax = (current.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) / dt;
        double ay = (current.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) / dt;
        double aOmga = (current.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond) / dt;

        lastSpeeds = current;
        lastAccelTime = now;

        return new double[]{ax, ay, aOmga};
    }

    public void restGyro() {
        double gyroAngle = !RobotCommon.getIsRed() ? 0 : 180;
        gyro.setYaw(gyroAngle);
    }

    public void resrtGyro180() {
        double gyroAngle = !RobotCommon.getIsRed() ? 180 : 0;
        gyro.setYaw(gyroAngle);
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

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                -sample.omega + headingController.calculate(pose.getRotation().getRadians(), -sample.heading));

        SmartDashboard.putNumber("traj/current heading", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("traj/heading error", sample.heading - pose.getRotation().getRadians());
        SmartDashboard.putNumber("traj/speeds omega", speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("traj/sample time", sample.getTimestamp());

        field.getObject("trajectory point #" + index).setPose(sample.getPose());
        index++;

        setVelocities(speeds);
    }

    public void resetTrajectory() {
        for (int i = index; i >= 0; i--) {
            field.getObject("trajectory point #" + i).setPose(Pose2d.kZero);
        }
        index = 0;
    }

    public void setDrivePower(double pow, int id) {
        modules[id].setDrivePower(pow);
    }

    public void setDrivePower(double pow) {
        for (int i = 0; i < 4; i++)
            setDrivePower(pow, i);
    }

    public double getMaxDriveVelocity() {
        return chassisConfig.maxDriveVelocity;
    }

    public double getMaxRotationalVelocity() {
        return chassisConfig.maxRotationalVelocity;
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

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public boolean isPassBamp() {
        return Math.toDegrees(gyro.getPitch().getValueAsDouble()) < 5
                || Math.toDegrees(gyro.getRoll().getValueAsDouble()) < 5;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPoseWithVelocity(double dt) {
        Pose2d currentPose = getPose();
        ChassisSpeeds currentSpeeds = getChassisSpeedsFieldRel();
        return new Pose2d(
                currentPose.getX() + (currentSpeeds.vxMetersPerSecond * dt),
                currentPose.getY() + (currentSpeeds.vyMetersPerSecond * dt),
                currentPose.getRotation().plus(new Rotation2d(currentSpeeds.omegaRadiansPerSecond * dt)));
    }

    public void setRotateToHub() {
        this.isRotateToHub = !isRotateToHub;
    }

    public void setVelocities(ChassisSpeeds speeds) {
        SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public Translation2d getVelocityAsVector() {
        return new Translation2d(
                getChassisSpeedsFieldRel().vxMetersPerSecond,
                getChassisSpeedsFieldRel().vyMetersPerSecond);
    }

    public void setRobotRelSpeedsWithAccel(ChassisSpeeds speeds) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGyroAngle());
        setVelocities(fieldSpeeds);
    }

    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
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

    public ChassisSpeeds getRobotRelVelocities() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsFieldRel(), getGyroAngle());
    }

    public void setRobotRelVelocities(ChassisSpeeds speeds) {
        SwerveModuleState[] states = wpilibKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setDriveVelocities(double velocity) {
        setDriveVelocities(new double[]{velocity, velocity, velocity, velocity});
    }

    public Rotation2d getGyroAngle() {
        gyroYawStatus.refresh();
        if (gyroYawStatus.getStatus() == StatusCode.OK) {
            lastGyroYaw = new Rotation2d(gyroYawStatus.getValue());
        }
        return lastGyroYaw;
    }

    public double getGyroAngularVelocity() {
        gyroAngularVelocityStatus.refresh();
        if (gyroAngularVelocityStatus.getStatus() == StatusCode.OK) {
            lastGyroAngularVelocity = gyroAngularVelocityStatus.getValue().in(Units.RadiansPerSecond);
        }
        return lastGyroAngularVelocity;
    }

    public void setModuleState(SwerveModuleState state) {
        setModuleStates(new SwerveModuleState[]{state, state, state, state});
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroAngle(), getModulePositions());

        SmartDashboard.putNumber("chassis/gyro angle", getGyroAngle().getDegrees());

        field.setRobotPose(getPose());
        
        fieldOdmetry.setRobotPose(DemaciaOdometry.getOdometryInstance(modulePositions).getPose2d());
    }

    public Pose2d getFuturePose(double dtSeconds) {
        return getPose().exp(new Twist2d(
                getChassisSpeedsFieldRel().vxMetersPerSecond * dtSeconds,
                getChassisSpeedsFieldRel().vyMetersPerSecond * dtSeconds,
                getChassisSpeedsFieldRel().omegaRadiansPerSecond * dtSeconds));
    }

    public ChassisSpeeds getChassisSpeedsRobotRel() {
        return demaciaKinematics.toChassisSpeeds(
                getModuleStates(),
                Math.toRadians(gyroYawStatus.getValueAsDouble()));
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

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] res = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = modules[i].getState();
        }
        return res;
    }

    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            gyro.setYaw(angle.getDegrees());
            // מאפסים גם את ה-PoseEstimator לפי הזווית החדשה
            poseEstimator.resetPosition(
                    angle,
                    getModulePositions(),
                    new Pose2d(getPose().getTranslation(), angle));
        }
    }

    public ChassisConfig getConfig() {
        return chassisConfig;
    }

    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }

    private void addStatus() {
        gyroYawStatus = gyro.getYaw();
        lastGyroYaw = new Rotation2d(gyroYawStatus.getValueAsDouble());
        gyroAngularVelocityStatus = gyro.getAngularVelocityZWorld();
        lastGyroAngularVelocity = gyroAngularVelocityStatus.getValue().in(Units.RadiansPerSecond);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[modules.length];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = modules[i].getModulePosition();
        }
        return arr;
    }
}