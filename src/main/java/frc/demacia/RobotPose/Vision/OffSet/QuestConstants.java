package frc.demacia.RobotPose.Vision.OffSet;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.demacia.RobotPose.Vision.visionConfigs.QuestConfig;

public final class QuestConstants {
    public static final String QUEST_NAME = "Quest";

    public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)     
    );

    public static final double STD_DEV_X = 0.02;
    public static final double STD_DEV_Y = 0.02;

    public static final QuestConfig QUEST_CONFIG = new QuestConfig(
        QUEST_NAME,
        ROBOT_TO_QUEST,
        VecBuilder.fill(STD_DEV_X, STD_DEV_Y, Double.POSITIVE_INFINITY)
    );
}
