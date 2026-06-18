// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision.subsystem;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.demacia.vision.Camera;

/** Add your docs here. */
public class Dvirs_ObjectPose extends SubsystemBase {

    private NetworkTable Table;

    private Camera objectCam;

    private double camObjectYaw;
    private double camObjectPitch;
    private double dist;

    private Translation2d robotToCam;
    private Translation2d[] previousPos = new Translation2d[3];
    private boolean tv = false;
    private Field2d fuleField = new Field2d();

    private InterpolatingDoubleTreeMap lut = new InterpolatingDoubleTreeMap();

    public Dvirs_ObjectPose(Camera objectCam) {
        super();
        lut.put(-8.0, 1.30);
        lut.put(-44.0, 0.22);
        lut.put(-26.0, 0.47);
        lut.put(-16.67, 0.78);
        lut.put(-12.0, 1.1);
        lut.put(-30.0, 0.4);
        lut.put(-25.0, 0.5);
        lut.put(-20.0, 0.6);
        // lut.put(-46.0 , 0.22);
        // lut.put(-37.0 , 0.33);
        // lut.put(-30.0 , 0.47);
        // lut.put(-21.5 , 0.7);
        // lut.put(-15.0 , 1.05);
        // lut.put(-11.75 , 1.32);

        Arrays.fill(previousPos, null);
        this.objectCam = objectCam;
        robotToCam = objectCam.getRobotToCamPosition().toTranslation2d();

        Table = NetworkTableInstance.getDefault().getTable(objectCam.getTableName());
        SmartDashboard.putString("objectCam.getTableName()", objectCam.getTableName());
        SmartDashboard.putData("Fuel", fuleField);

    }

    public void periodic() {
        updateValues();
        if (tv) {
            double yaw = Math.toRadians(camObjectYaw);
            double alpha = (camObjectPitch + objectCam.getPitch()) * Math.cos(yaw);
            yaw += Math.toRadians(objectCam.getYaw());
            // double alpha = Math.atan(Math.tan(Math.toRadians(camObjectPitch +
            // objectCam.getPitch()))/Math.cos(yaw));
            dist = lut.get(alpha);
            SmartDashboard.putNumberArray("FuleCam", new Double[] { Math.toDegrees(yaw), alpha, dist });
            SmartDashboard.putNumber("Fuel distance", dist);
            Translation2d robotToFuel = new Translation2d(robotToCam.getX() + dist * Math.cos(yaw),
                    robotToCam.getY() + dist * Math.sin(yaw));
            SmartDashboard.putNumber("robotToCam.getY()", robotToCam.getY());
            SmartDashboard.putNumber("robotToFuelx", robotToFuel.getX());
            SmartDashboard.putNumber("robotToFuely", robotToFuel.getY());
            // RobotCommon.fuelPosition = robotToFuel;
            // Translation2d fuelPos = RobotCommon.currentRobotPose.getTranslation().
            // plus(robotToFuel.rotateBy(RobotCommon.robotAngle));
            // } else if(Timer.getFPGATimestamp() - RobotCommon.fuelTime > 0.1) {
            // RobotCommon.fuelPosition = null;
        }
    }

    public boolean isObjectDetected() {
        return tv;
    }

    public void updateValues() {
        tv = Table.getEntry("tv").getDouble(0.0) != 0;
        if (tv) {
            camObjectPitch = Table.getEntry("ty").getDouble(0.0);
            camObjectYaw = (-Table.getEntry("tx").getDouble(0.0));
        } else {
            camObjectPitch = 0;
            camObjectYaw = 0;
        }
    }

}