// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.wpilib_kinmatics;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class wpilibKinmatics {
    private SwerveDriveKinematics kinematics;
    private double dt;

    public wpilibKinmatics(Translation2d[] moudlePose){
        kinematics = new SwerveDriveKinematics(moudlePose);
    }

    public SwerveModuleState[] toSwerveModuleStateWithFix(ChassisSpeeds speeds, SwerveModuleState[] currentModuleState, Rotation2d gyroAngle){
        SwerveModuleState[] swerveModuleStatesWpilib = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState[] swerveModuleStatesFix = new SwerveModuleState[swerveModuleStatesWpilib.length];


        for (int i = 0; i < swerveModuleStatesWpilib.length; i++) {
            swerveModuleStatesFix[i].angle = new Rotation2d(swerveModuleStatesWpilib[i].angle.getRadians() + (currentModuleState[i].angle.getRadians()) - gyroAngle.getRadians() * dt);
        }
        return swerveModuleStatesFix;
    }

    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds){
        return kinematics.toSwerveModuleStates(speeds);
    }

    public SwerveDriveKinematics kinematics(){
        return kinematics;
    }
}
