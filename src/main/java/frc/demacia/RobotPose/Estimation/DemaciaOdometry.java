// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.RobotPose.Estimation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public class DemaciaOdometry {

    private Pose2d pose;
    /** Field heading minus raw gyro reading, set on reset so the reset heading is kept. */
    private Rotation2d gyroOffset = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions;
    private Translation2d[] moduleDisplacements;
    private double[] modulesWeights;
    /** Module locations relative to the robot centre (meters). */
    private final Translation2d[] moduleLocations;

    public DemaciaOdometry(SwerveModulePosition[] initialModulePositions, Translation2d[] moduleLocations) {
        this.moduleLocations = moduleLocations;
        this.lastModulePositions = new SwerveModulePosition[initialModulePositions.length];
        this.modulesWeights = new double[initialModulePositions.length];
        this.moduleDisplacements = new Translation2d[initialModulePositions.length];

        for (int i = 0; i < initialModulePositions.length; i++) {
            lastModulePositions[i] = initialModulePositions[i].copy();
            modulesWeights[i] = 1.0 / initialModulePositions.length;
            moduleDisplacements[i] = Translation2d.kZero;
        }
        this.pose = Pose2d.kZero;
    }

    /**
     * Updates the odometry with new gyro and module readings.
     *
     * <p>The combined robot-relative displacement (weighted average of per-module arc
     * displacements) is packaged into a robot-relative Twist2d together with the gyro's
     * dtheta, and {@code pose} is advanced via {@code pose.exp(twist)} - the same
     * primitive WPILib's own pose estimators use to turn a twist into a pose. This is
     * deliberate: the twist returned to the caller is the exact twist used to update
     * {@code pose} internally, so replaying it externally via Pose2d.exp() from the same
     * starting pose is guaranteed (by construction, not by small-angle approximation) to
     * reproduce this method's own internal pose update - including when a single tick
     * combines translation and rotation. 
     *
     * @param gyroAngle       The current gyro heading.
     * @param modulePositions The current swerve module positions.
     * @return The robot-relative Twist2d representing the motion applied by this update.
     *         
     */
    public Twist2d updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        for (int i = 0; i < modulePositions.length; i++) {
            moduleDisplacements[i] = calculateModuleDisplacement(lastModulePositions[i], modulePositions[i]);
        }

        Rotation2d previousRotation = pose.getRotation();
        double dtheta = MathUtil.angleModulus(gyroAngle.plus(gyroOffset).minus(previousRotation).getRadians());

        Translation2d robotDisplacementRobotRelative = calculateRobotDisplacement(moduleDisplacements, dtheta);

        Twist2d twist = new Twist2d(robotDisplacementRobotRelative.getX(),
                robotDisplacementRobotRelative.getY(), dtheta);

        pose = pose.exp(twist);

        for (int i = 0; i < lastModulePositions.length; i++) {
            lastModulePositions[i] = modulePositions[i].copy();
        }

        return twist;
    }

    private Translation2d calculateModuleDisplacement(SwerveModulePosition lastPosition,
            SwerveModulePosition currentPosition) {
        double arcLength = currentPosition.distanceMeters - lastPosition.distanceMeters;
        double deltaAngle = MathUtil.angleModulus(currentPosition.angle.getRadians() - lastPosition.angle.getRadians());

        if (Math.abs(Math.toDegrees(deltaAngle)) < 1E-6)
            return new Translation2d(arcLength, currentPosition.angle); // case for almost straight line

        double centralAngle = deltaAngle;
        double radius = arcLength / centralAngle;
        double chordLength = 2 * radius * Math.sin(centralAngle * 0.5);
        Rotation2d chordAngle = lastPosition.angle.plus(new Rotation2d(centralAngle * 0.5));

        return new Translation2d(chordLength, chordAngle);
    }

    /**
     * Module i moves (dx - dtheta * y_i, dy + dtheta * x_i), so the weighted mean is
     * (dx, dy) + dtheta * (-y_w, x_w) where (x_w, y_w) is the weighted module centroid.
     * The rotation term is removed so non-uniform weights don't turn rotation into translation.
     */
    private Translation2d calculateRobotDisplacement(Translation2d[] moduleDisplacements, double dtheta) {
        double x = 0;
        double y = 0;
        for (int i = 0; i < moduleDisplacements.length; i++) {
            x += (moduleDisplacements[i].getX() + dtheta * moduleLocations[i].getY()) * modulesWeights[i];
            y += (moduleDisplacements[i].getY() - dtheta * moduleLocations[i].getX()) * modulesWeights[i];
        }
        return new Translation2d(x, y);
    }

    /**
     * Resets the pose. The gyro reading at the moment of the reset is needed so the new
     * heading is kept on the next update instead of being replaced by the raw gyro.
     *
     * @param newPose   The new field pose.
     * @param gyroAngle The raw gyro reading that corresponds to newPose's heading.
     */
    public void resetPose(Pose2d newPose, Rotation2d gyroAngle) {
        this.pose = newPose;
        this.gyroOffset = newPose.getRotation().minus(gyroAngle);
    }

    public Pose2d getOdometryPose() {
        return this.pose;
    }

    /**
     * Sets a module's weight in the robot-displacement average, then renormalizes every
     * other module's weight so the full array still sums to 1. 
     *
     * @param index     The index of the module whose weight is being changed.
     * @param newWeight The new weight for that module, clamped to [0, 1].
     */
    public void changeModuleWeight(int index, double newWeight) {
        double clampedNewWeight = MathUtil.clamp(newWeight, 0, 1);

        double remainingWeightSum = 0;
        for (int i = 0; i < modulesWeights.length; i++) {
            if (i != index) {
                remainingWeightSum += modulesWeights[i];
            }
        }

        double weightToDistribute = 1.0 - clampedNewWeight;
        modulesWeights[index] = clampedNewWeight;

        for (int i = 0; i < modulesWeights.length; i++) {
            if (i != index) {
                modulesWeights[i] = remainingWeightSum > 0
                        ? (modulesWeights[i] / remainingWeightSum) * weightToDistribute
                        : weightToDistribute / (modulesWeights.length - 1);
            }
        }
    }
}