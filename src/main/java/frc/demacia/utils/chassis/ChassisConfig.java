package frc.demacia.utils.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.vision.Camera;
import frc.demacia.vision.TagPose;

/**
 * Configuration class for swerve drive chassis using Pigeon 1 (Legacy).
 * <p>Contains all module configurations, physical dimensions, and motion constraints.</p>
 */
public class ChassisConfig {
    public final String name;
    public final SwerveModuleConfig[] swerveModuleConfig;
    
    // הגדרת ה-ID של ה-Pigeon 1 במקום החיישן עצמו
    public final int pigeonCanId; 
    public final boolean isPigeonAttachedToTalon; // האם הוא מחובר ל-Talon SRX או ישירות ל-CAN

    public TagPose[] tags;
    public Camera objectCamera;

    public double cycleDt = 0.02;
    public double maxLinearAccel = 10;
    public double maxOmegaVelocity = Math.toRadians(540);
    public double maxRadialAccel = 6;
    public double maxRadius = 0.4;
    public double minOmegaDiff = Math.toRadians(20);
    public double maxDeltaVelocity = maxLinearAccel * cycleDt;
    public double maxVelocityToIgnoreRadius = maxRadius * maxOmegaVelocity;
    public double minVelocity = 1.5;
    public double maxDriveVelocity = 5;
    public double maxRotationalVelocity = 4;
    public Translation2d[] modulePositions;
    
    public ChassisConfig(String name, SwerveModuleConfig[] swerveModuleConfigs, int pigeonCanId, TagPose[] tags) {
        this.name = name;
        this.swerveModuleConfig = swerveModuleConfigs;
        this.pigeonCanId = pigeonCanId;
        this.isPigeonAttachedToTalon = false;
        this.tags = tags;
        this.modulePositions = new Translation2d[swerveModuleConfigs.length];

        for (int i = 0; i < swerveModuleConfigs.length; i++) {
            modulePositions[i] = swerveModuleConfigs[i].position;
        }
    }

    public ChassisConfig withCycleDt(double cycleDt){
        this.cycleDt = cycleDt;
        return this;
    }

    public ChassisConfig withMaxLinearAccel(double maxLinearAccel){
        this.maxLinearAccel = maxLinearAccel;
        return this;
    }

    public ChassisConfig withMaxOmegaVelocity(double maxOmegaVelocity){
        this.maxOmegaVelocity = maxOmegaVelocity;
        return this;
    }

    public ChassisConfig withMaxRadialAccel(double maxRadialAccel){
        this.maxRadialAccel = maxRadialAccel;
        return this;
    }

    public ChassisConfig withMaxRadius(double maxRadius){
        this.maxRadius = maxRadius;
        return this;
    }

    public ChassisConfig withMinOmegaDiff(double minOmegaDiff){
        this.minOmegaDiff = minOmegaDiff;
        return this;
    }

    public ChassisConfig withMaxDeltaVelocity(double maxDeltaVelocity){
        this.maxDeltaVelocity = maxDeltaVelocity;
        return this;
    }

    public ChassisConfig withMaxVelocityToIgnoreRadius(double maxVelocityToIgnoreRadius){
        this.maxVelocityToIgnoreRadius = maxVelocityToIgnoreRadius;
        return this;
    }

    public ChassisConfig withMinVelocity(double minVelocity){
        this.minVelocity = minVelocity;
        return this;
    }

    public ChassisConfig withMaxDriveVelocity(double maxDriveVelocity){
        this.maxDriveVelocity = maxDriveVelocity;
        return this;
    }

    public ChassisConfig withMaxRotationalVelocity(double maxRotationalVelocity){
        this.maxRotationalVelocity = maxRotationalVelocity;
        return this;
    }

    public ChassisConfig withObjectCamera(Camera objectCamera){
        this.objectCamera = objectCamera;
        return this;
    }
}