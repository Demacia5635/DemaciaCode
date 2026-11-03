// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

<<<<<<< HEAD
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
=======
import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
>>>>>>> wpilib_kinmatics
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.demacia.vision.utils.Vision;
import frc.demacia.vision.utils.VisionConstants;

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

<<<<<<< HEAD
    private Rotation2d[] lastAngles;
    private SwerveDrivePoseEstimator poseEstimator;

=======
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

    Rotation2d gyroAngle;

    OdometryObservation observation;
    public Translation2d[] modulePositions;
>>>>>>> wpilib_kinmatics
    private Chassis(ChassisConfig chassisConfig) {
        setName(getName());
        this.chassisConfig = chassisConfig;
        modules = new SwerveModule[4];
        modulePositions = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(chassisConfig.swerveModuleConfig[i]);
            modulePositions[i] = chassisConfig.swerveModuleConfig[i].position;
<<<<<<< HEAD
            if (modulePositions[i].getNorm() == 0) {
                Log.alert(getName() + " module location on the robot is 0");
            }
=======
        }
>>>>>>> wpilib_kinmatics

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
    }

    public void addLog() {
        Log.putData("chassis/gyro angle", () -> getGyroAngle().getDegrees());

        SmartDashboard.putData("chassis/set coast",
                new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData("chassis/set brake",
                new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset moduls", new InstantCommand(()-> resetMudolse()).ignoringDisable(true));
    }

<<<<<<< HEAD
=======
    public void setDrivePower(double pow, int id) {
        modules[id].setDrivePower(pow);
    }

    public void setDrivePower(double pow) {
        for (int i = 0; i < 4; i++)
            setDrivePower(pow, i);
    }

    public RobotPose getRoobotPose(){
        return RobotPose.getInstance();
    }

    /**
     * Checks all module electronics for faults and logs them.
     */
>>>>>>> wpilib_kinmatics
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

<<<<<<< HEAD
    public double getGyroAngularVelocity() {
        return gyro.getZVelocity();
=======
    /**
     * Sets chassis velocities without acceleration limiting.
     * 
     * <p>
     * Applies discrete kinematics for accurate odometry.
     * Use this for precise path following where acceleration is pre-profiled.
     * </p>
     * 
     * @param speeds Desired chassis speeds (field-relative)
     */

    public void setVelocities(ChassisSpeeds speeds) {

        SwerveModuleState[] states = demaciaKinematics
                .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle()));
        // SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStatesWithLimit(
        // speeds,
        // getChassisSpeedsFieldRel(),
        // getGyroAngle());
        setModuleStates(states);
>>>>>>> wpilib_kinmatics
    }

    public void setGyroYaw(Rotation2d angle) {
        if (angle != null) {
            gyro.setYaw(angle.getDegrees());
        }
    }

    public void stop() {
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

    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

<<<<<<< HEAD
    public double getMaxDriveVelocity() {
        return chassisConfig.maxDriveVelocity;
    }

    public double getMaxRotationalVelocity() {
        return chassisConfig.maxRotationalVelocity;
    }
=======
    @Override
    public void periodic() {
        // updateCommon(); TODO: ROBOT COMOON

        observation = new OdometryObservation(
                Timer.getFPGATimestamp(),
                getGyroAngle(),
                getModulePositions());

        RobotPose.getInstance().update(observation);
        field.setRobotPose(getPose());
        // field.getObject("Turret").setPose(new Pose2d(RobotCommon.getCurrentRobotPose().getTranslation()
        //         .plus(TurretConstants.TURRET_POSITION_ON_ROBOT.rotateBy(RobotCommon.getRobotAngle())),
        //         Rotation2d.fromRadians(RobotCommon.getRobotAngle().getRadians()
        //                 + MathUtil.angleModulus(Turret.getInstance().getTurretAngle()))));
        // field.getObject("estimation").setPose(ShooterUtils.computeFuturePosition(getChassisSpeedsFieldRel(), RobotCommon.getCurrentRobotPose(), 0.1));
    }
    //TODO: ROBOT COMMON
    // public void updateCommon() {
    //     RobotCommon.setRobotAngle(getGyroAngle());
    //     RobotCommon.setCurrentRobotPose(getPose());
    //     RobotCommon.setFieldRelativeSpeeds(getChassisSpeedsFieldRel());
    //     RobotCommon.setFutureRobotPose(getFuturePose(0.2));
    // }
>>>>>>> wpilib_kinmatics

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
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
<<<<<<< HEAD
=======

    /**
     * Stops all swerve modules immediately.
     */
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
>>>>>>> wpilib_kinmatics
}