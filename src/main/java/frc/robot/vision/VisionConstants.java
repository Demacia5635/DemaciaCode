package frc.robot.vision;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.demacia.RobotPose.Vision.VisionConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera2dConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.QuestConfig;

public class VisionConstants {

    public static final String BACK_2D_NAME = "back 2d";
    public static final Transform3d BACK_2D_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),  // TODO
        new Rotation3d(0.0, 0.0, 0.0));  // TODO
    public static final Matrix<N3, N1> BACK_2D_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.0, 0.0, 0.0 }));  // TODO
    public static final LimelightTagCamera2dConfig BACK_2D_CONFIG = new LimelightTagCamera2dConfig(BACK_2D_NAME, BACK_2D_OFFSET, BACK_2D_STD);

    public static final String BACK_3D_NAME = "back 3d";
    public static final Transform3d BACK_3D_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),  // TODO
        new Rotation3d(0.0, 0.0, 0.0));  // TODO
    public static final Matrix<N3, N1> BACK_3D_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.0, 0.0, 0.0 }));  // TODO
    public static final LimelightTagCamera3dConfig BACK_3D_CONFIG = new LimelightTagCamera3dConfig(BACK_3D_NAME, BACK_3D_OFFSET, BACK_3D_STD);

    public static final String BACK_3D_2_NAME = "back 3d 2";
    public static final Transform3d BACK_3D_2_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0), 
        new Rotation3d(0.0, 0.0, 0.0)); 
    public static final Matrix<N3, N1> BACK_3D_2_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.0, 0.0, 0.0 })); 
    public static final LimelightTagCamera3dConfig BACK_3D_2_CONFIG = new LimelightTagCamera3dConfig(BACK_3D_2_NAME, BACK_3D_2_OFFSET, BACK_3D_2_STD);

    public static final String QUEST_NAME = "quest";
    public static final Transform3d QUEST_OFFSET = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0), 
        new Rotation3d(0.0, 0.0, 0.0)); 
    public static final Matrix<N3, N1> QUEST_STD = new Matrix<>(new SimpleMatrix(new double[] { 0.0, 0.0, 0.0 })); 
    public static final QuestConfig QUEST_CONFIG = new QuestConfig(QUEST_NAME, QUEST_OFFSET, QUEST_STD);

    public static final VisionConfig visionConfig = new VisionConfig()
        .addSource(BACK_2D_CONFIG)
        .addSource(BACK_3D_CONFIG)
        .addSource(BACK_3D_2_CONFIG)
        .addSource(QUEST_CONFIG)
        ;

    // VisionConfig visionConfig = new VisionConfig(TWO_D_CONFIG, BACK_CONFIG, QUEST_CONFIG);
}
