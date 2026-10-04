package frc.robot.vision;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.demacia.RobotPose.Vision.VisionConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.QuestConfig;

public class VisionConstants {

    public static final String QUEST_NAME = "quest";
    public static final Transform3d QUEST_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),  // TODO
        new Rotation3d(0.0, 0.0, 0.0));  // TODO
    public static final Matrix<N3, N1> QUEST_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.03, 0.03, 0.0 })); 
    public static final QuestConfig QUEST_CONFIG = new QuestConfig(QUEST_NAME, QUEST_OFFSET, QUEST_STD);

    public static final String ROBORIO_NAME = "roboio";
    public static final Transform3d ROBORIO_OFFSET = new Transform3d(
        new Translation3d(-0.325, 0.295, 0.305),  // TODO
        new Rotation3d(0.0, Math.toRadians(20.0), Math.toRadians(-90.0)));  // TODO
    public static final Matrix<N3, N1> ROBORIO_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.03, 0.03, 0.0 })); 
    public static final LimelightTagCamera3dConfig ROBORIO_CONFIG = new LimelightTagCamera3dConfig(ROBORIO_NAME, ROBORIO_OFFSET, ROBORIO_STD);

    public static final VisionConfig visionConfig = new VisionConfig()
        // .addSource(QUEST_CONFIG)
        // .addSource(ROBORIO_CONFIG)
        ;
}
