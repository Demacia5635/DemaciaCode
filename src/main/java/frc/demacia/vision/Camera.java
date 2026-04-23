// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision;

import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */

public class Camera {

    private String name;
    private Translation3d robotToCamPosition;
    private double pitch;
    private double yaw;
    private String tableName;
    private Enum<?> cameraType;

    public Camera(String name, Translation3d robotToCamPosition, double pitch, double yaw, Enum<?> cameraType) {
        this.name = name;
        this.robotToCamPosition = robotToCamPosition;
        this.pitch = pitch;
        this.yaw = yaw;
        this.cameraType = cameraType;
        this.tableName = "limelight-"+name;
    }

    public Translation3d getRobotToCamPosition() {
        return robotToCamPosition;
    }

    public double getHeight() {
        return robotToCamPosition.getZ();
    }

    public double getPitch() {
        return this.pitch;
    }

    public double getYaw() {
        return this.yaw;
    }

    public String getName() {
        return this.name;
    }

    public String getTableName() {
        return this.tableName;
    }
    public Enum<?> getCameraType(){return this.cameraType;}
}