package frc.demacia.RobotPose.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.RobotPose.Vision.visionConfigs.BaseVisionSourceConfig;
import frc.demacia.utils.elastic.ElasticGenerator;

public abstract class BaseVisionSource implements VisionSource, Sendable {
    protected final String name;
    protected Transform3d offset;
    protected Matrix<N3, N1> std;

    private Field2d field;

    public BaseVisionSource(BaseVisionSourceConfig config) {
        name = config.name;
        offset = config.offset;
        std = config.std;
        field = new Field2d();

        addLog();
        ElasticGenerator.getInstance().registerVisionSource(this);
        SmartDashboard.putData("vision/" + name, this);
    }

    protected void addLog() {
        SmartDashboard.putData("vision/" + name + "/field", field);
    }

    public String getName() {
        return name;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("is Connected", () -> isConnected(), null);
    }
}
