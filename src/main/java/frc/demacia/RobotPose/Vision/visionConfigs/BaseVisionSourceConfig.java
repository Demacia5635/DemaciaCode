package frc.demacia.RobotPose.Vision.visionConfigs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.demacia.RobotPose.Vision.VisionSource;
import frc.demacia.RobotPose.Vision.VisionTypes.LimelightTagCamera2d;
import frc.demacia.RobotPose.Vision.VisionTypes.LimelightTagCamera3d;
import frc.demacia.RobotPose.Vision.VisionTypes.Quest;


public abstract class BaseVisionSourceConfig {
    public static enum VisionSourceType {
        Limelight2d {
            @Override
            public VisionSource create(BaseVisionSourceConfig config) {
                return new LimelightTagCamera2d((LimelightTagCamera2dConfig) config);
            }
        },
        Limelight3d {
            @Override
            public VisionSource create(BaseVisionSourceConfig config) {
                return new LimelightTagCamera3d((LimelightTagCamera3dConfig) config);
            }
        },
        Quest {
            @Override
            public VisionSource create(BaseVisionSourceConfig config) {
                return new Quest((QuestConfig) config);
            }
        };

        public abstract VisionSource create(BaseVisionSourceConfig config);
    };

    public String name;
    public Transform3d offset;
    public Matrix<N3, N1> std;
    public VisionSourceType visionSourceType = VisionSourceType.Limelight2d;

    public BaseVisionSourceConfig(String name, Transform3d offset, Matrix<N3, N1> std) {
        this.name = name;
        this.offset = offset;
        this.std = std;
    }

    public VisionSourceType getVisionSourceType() {
        return visionSourceType;
    }
}