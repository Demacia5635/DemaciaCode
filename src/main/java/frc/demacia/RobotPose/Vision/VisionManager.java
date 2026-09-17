package frc.demacia.RobotPose.Vision;

import java.util.ArrayList;
import java.util.List;

import frc.demacia.RobotPose.Vision.visionConfigs.BaseVisionSourceConfig;

public final class VisionManager {

    private static VisionManager instance;

    private final List<VisionSource> sources;

    private VisionManager() {
        this.sources = new ArrayList<>();
    }

    public VisionManager addSource(BaseVisionSourceConfig config) {
        sources.add(config.getVisionSourceType().create(config));
        return this;
    }

    /** Returns the configured, immutable list of all registered VisionSources (including Quest, if present). */
    public List<VisionSource> getSources() {
        return sources;
    }

    /**
     * Returns the singleton instance, or null if initialize(...) has not been called yet.
     */
    public static synchronized VisionManager getInstance() {
        if (instance == null) {
            instance = new VisionManager();
        }
        return instance;
    }
}