// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.Paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Waypoint extends Pose2d {
    private double maxCornerSpeed = PathsConstants.MAX_VELOCITY; // m/s

    public Waypoint(Pose2d pose){
        this(pose.getX(), pose.getY(), pose.getRotation());
        
    }
    public Waypoint(Translation2d translation, Rotation2d heading) {
        this(translation.getX(), translation.getY(), heading);
        
    }

    public Waypoint(double x, double y, Rotation2d heading) {
        super(x, y, heading);
        
    }

    public Waypoint withMaxCornerSpeed(double maxCornerSpeed) {
        this.maxCornerSpeed = maxCornerSpeed;
        return this;
    }

    public double getMaxCornerSpeed() {
        return maxCornerSpeed;
    }

}
