package frc.demacia.RobotPose.Vision;

import java.util.ArrayList;
import java.util.List;
import frc.demacia.RobotPose.Vision.visionConfigs.BaseVisionSourceConfig;

public class VisionConfig {
    private final List<VisionSource> sources;

    public VisionConfig() {
        this.sources = new ArrayList<>();
    }

    public VisionConfig(BaseVisionSourceConfig... sourceConfigs) {
        this();

        for (BaseVisionSourceConfig sourceConfig : sourceConfigs) {
            sources.add(sourceConfig.getVisionSourceType().create(sourceConfig));
        }
    }

    public VisionConfig addSource(BaseVisionSourceConfig config) {
        sources.add(config.getVisionSourceType().create(config));
        return this;
    }

    /** Returns the configured, immutable list of all registered VisionSources (including Quest, if present). */
    public List<VisionSource> getSources() {
        return sources;
    }
}