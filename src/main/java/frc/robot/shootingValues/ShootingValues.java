package frc.robot.shootingValues;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.utils.RobotCommon;
import frc.demacia.utils.chassis.Chassis;
import static frc.robot.shootingValues.ShootingValuesConstants.*;

public class ShootingValues {
    private static ShootingValues instance;

    private Chassis chassis;

    private DoubleSupplier targetBallSpeedSupplier;
    private DoubleSupplier targetBallPitchSupplier;
    private DoubleSupplier targetBallYawSupplier;

    private ShootingValuesRecord shootingValuesRecord;

    private ShootingValues(){
        chassis = Chassis.getInstance();
        targetBallSpeedSupplier = () -> {
            return LOOK_UP_TABLE.get(distanceFromHubAfterTime(PREDICTING_TIME))[0] * MOTOR_VEL_TO_BALL_VEL;
        };
        targetBallPitchSupplier = () -> {
            return LOOK_UP_TABLE.get(distanceFromHubAfterTime(PREDICTING_TIME))[1];
        };
        targetBallYawSupplier = () -> {
            return angleFromHubAfterTime(PREDICTING_TIME);
        };

        shootingValuesRecord = new ShootingValuesRecord(0, 0, 0);
    }

    public static ShootingValues getInstance() {
        if (instance == null) {
            instance = new ShootingValues();
        }
        return instance;
    }


    private Translation2d getTurretFuturePosition(double sec) {
        Pose2d futurePose = RobotPose.getInstance().getFuturePose(sec);
        double mountingAngle = TURRET_POSE_ON_ROBOT.getAngle().getRadians() + futurePose.getRotation().getRadians();
        
        return futurePose.getTranslation().plus(new Translation2d(
            TURRET_POSE_ON_ROBOT.getNorm() * Math.cos(mountingAngle),
            TURRET_POSE_ON_ROBOT.getNorm() * Math.sin(mountingAngle)
        ));
    }

    public double distanceFromHubAfterTime(double sec) {
        return getHubPos().getDistance(getTurretFuturePosition(sec));
    }

    public double angleFromHubAfterTime(double sec) {
        Translation2d turretPos = getTurretFuturePosition(sec);
        double turretAngle = getHubPos().minus(turretPos).getAngle().getRadians();
    
        return turretAngle;
    }

    private Translation2d getHubPos() {
        return RobotCommon.getIsRed() ? HUB_POSE_RED : HUB_POSE_BLUE;
    }

    public void updateShootingValues() {
        double targetSpeed = targetBallSpeedSupplier.getAsDouble();
        double targetBallPitch = targetBallPitchSupplier.getAsDouble();
        double targetBallYaw = targetBallYawSupplier.getAsDouble();
        
        Pose2d futurePose = RobotPose.getInstance().getFuturePose(PREDICTING_TIME);

        Translation3d endValues = new Translation3d(
            targetSpeed * Math.sin(targetBallPitch) * Math.cos(targetBallYaw),
            targetSpeed * Math.sin(targetBallPitch) * Math.sin(targetBallYaw),
            targetSpeed * Math.cos(targetBallPitch)
        );
        
        ChassisSpeeds speeds = chassis.getChassisSpeedsFieldRel();
        Translation3d chassisSpeed = new Translation3d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond, 
            0
        );
        Translation3d tangentialVelocity = new Translation3d(
            -speeds.omegaRadiansPerSecond*Math.sin(TURRET_POSE_ON_ROBOT.getAngle().getRadians() + futurePose.getRotation().getRadians()) * TURRET_POSE_ON_ROBOT.getNorm(), 
            speeds.omegaRadiansPerSecond*Math.cos(TURRET_POSE_ON_ROBOT.getAngle().getRadians() + futurePose.getRotation().getRadians()) * TURRET_POSE_ON_ROBOT.getNorm(), 
            0
        );
        chassisSpeed = chassisSpeed.plus(tangentialVelocity);

        Translation3d shooterValues = endValues.minus(chassisSpeed);
        
        double ShooterSpeed = shooterValues.getNorm();
        if (ShooterSpeed < 1e-6) {
            shootingValuesRecord = new ShootingValuesRecord(
                0,
                targetBallPitch,
                MathUtil.angleModulus(targetBallYaw - futurePose.getRotation().getRadians())
            );
            return;
        }

        shooterValues = shooterValues.minus(new Translation3d(
            0, 
            0, 
            SPIN_CORRECTION_GAIN * (ShooterSpeed - targetSpeed) * targetSpeed *
            targetSpeed * Math.cos(Math.asin(shooterValues.getZ() / ShooterSpeed)) *  distanceFromHubAfterTime(PREDICTING_TIME)
        ));
        ShooterSpeed = shooterValues.getNorm();

        shootingValuesRecord = new ShootingValuesRecord(
            ShooterSpeed / MOTOR_VEL_TO_BALL_VEL, 
            Math.acos(MathUtil.clamp(shooterValues.getZ() / ShooterSpeed, -1, 1)),
            MathUtil.angleModulus(shooterValues.toTranslation2d().getAngle().getRadians() - futurePose.getRotation().getRadians())
        );
    }

    public ShootingValuesRecord getShootingValues() {
        return shootingValuesRecord;
    }
}