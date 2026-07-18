package frc.demacia.vision.subsystem;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.vision.TagPose;

import static frc.demacia.vision.VisionConstants.*;

public class Vision extends SubsystemBase{
    private static Vision vision;

    private ArrayList<TagPose> tags;
    private Quest quest;
    private boolean hasUpdatedQuestIntialPose;
    private boolean hasQuestDisconnected;

    private Vision() {
        this.tags = new ArrayList<>();
        for (TagPose pose : TAGS) {
            tags.add(pose);
        }
        quest = new Quest(QUEST_OFFSET);
        hasUpdatedQuestIntialPose = false;
        hasQuestDisconnected = true;
    }

    public static Vision getInstance() {
        if (vision == null) {
            vision = new Vision();
        }
        return vision;
    }

    public void addTag(TagPose tag) {
        tags.add(tag);
    }

    public ArrayList<TagPose> getTags() {
        return tags;
    }

    public Quest getQuest() {
        return quest;
    }

    public boolean isSeeTag() {
        for(TagPose t : tags){
            if(t.isSeeTag()) return true;
        }
        return false;
    }

    public void setQuestPose() {
        quest.setQuestPose(new Pose3d(getTagsPoseEstimation()));
        hasUpdatedQuestIntialPose = true;
    }

    private double getCollectedConfidence() {
        double confidence = 0;
        for (TagPose tag : tags) {
            if (tag.getRobotPose2d() != null) {
                confidence += tag.getPoseEstemationConfidence();
            }
        }
        return confidence;
    }

    private double normalizeConfidence(double confidence) {
        return getCollectedConfidence() == 0 ? 0 : confidence * (1d / getCollectedConfidence());
    }
    
    public Rotation2d getRobotAngle(){
        for (TagPose tag : tags) {
            if (tag.getRobotPose2d() != null) {
                return Rotation2d.fromDegrees(tag.getAngle());
            }
        }
        return null;
    }

    public void addVisionMeasurement(Translation2d PoseEstimation, Matrix<N3, N1> STD , Rotation2d gyroAngle) {
        Chassis.getInstance().getPoseEstimate().setVisionMeasurementStdDevs(STD);
        Chassis.getInstance().getPoseEstimate().addVisionMeasurement(
                new Pose2d(PoseEstimation.getX(), PoseEstimation.getY(), gyroAngle),
                Timer.getFPGATimestamp() - 0.05);
    }

    public Pose2d getTagsPoseEstimation() {
        double x = 0;
        double y = 0;
        double confidence = 0;
        for (TagPose tag : tags) {
            Pose2d pose2d = tag.getRobotPose2d();
            if (pose2d == null)
                continue;
            confidence = normalizeConfidence(tag.getPoseEstemationConfidence());
            x += pose2d.getX() * confidence;
            y += pose2d.getY() * confidence;
        }
        return new Pose2d(x, y, Chassis.getInstance().getGyroAngle());
    }

    @Override
    public void periodic() {
        if (hasUpdatedQuestIntialPose && quest.isConnected()) {
            addVisionMeasurement(quest.getRobotPose2d().getTranslation(), QUEST_STD ,Chassis.getInstance().getGyroAngle());
        }
        if (isSeeTag()) {
            addVisionMeasurement(getTagsPoseEstimation().getTranslation(), LIMELIGHT_STD ,Chassis.getInstance().getGyroAngle());
            if (hasQuestDisconnected && quest.isConnected()) {
                setQuestPose();
                hasQuestDisconnected = false;
            }
        }
        if (!hasQuestDisconnected && !quest.isConnected()) {
            hasQuestDisconnected = true;
        }
    }
}
