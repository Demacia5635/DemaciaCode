package frc.demacia.utils.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.demacia.kinematics.DemaciaKinematics;

public class ChassisCommon {
    Chassis chassis;
    DemaciaKinematics kinematics;
    public ChassisCommon(Chassis chassis, DemaciaKinematics kinematics){
        this.chassis = chassis;
        this.kinematics = kinematics;
    }

    public ChassisSpeeds getWantedChassisSpeed(){
        return chassis.getTargetVel();
    }

    public ChassisSpeeds getCurrentChassisSpeeds(){
        return chassis.getChassisSpeedsFieldRel();
    }

    public SwerveModuleState[] getSwerveModuleState(){
        return chassis.getModuleStates();
    } 

    public Pose2d getPose(){
        return chassis.getPose();
    }
}
