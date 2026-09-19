package frc.robot.vision;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.demacia.RobotPose.Vision.VisionConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera2dConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.LimelightTagCamera3dConfig;
import frc.demacia.RobotPose.Vision.visionConfigs.QuestConfig;

public class VisionConstants {
    public static final Matrix<N3, N1> visionStd = new Matrix<>(new SimpleMatrix(new double[] { 0.3, 0.3, 0 }));
    public static final LimelightTagCamera2dConfig sourceConfig1 = new LimelightTagCamera2dConfig("2d", new Transform3d(), visionStd);
    public static final LimelightTagCamera3dConfig sourceConfig2 = new LimelightTagCamera3dConfig("back", new Transform3d(), visionStd);
    public static final QuestConfig sourceConfig3 = new QuestConfig("quest", new Transform3d(), visionStd);

    public static final VisionConfig visionConfig = new VisionConfig()
        .addSource(sourceConfig1)
        .addSource(sourceConfig2)
        .addSource(sourceConfig3)
        ;

    // VisionConfig visionConfig = new VisionConfig(sourceConfig1, sourceConfig2, sourceConfig3);
}
