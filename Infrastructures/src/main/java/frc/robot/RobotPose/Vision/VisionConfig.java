package frc.robot.RobotPose.Vision;

import java.util.ArrayList;
import java.util.List;
import frc.robot.RobotPose.Vision.visionConfigs.BaseVisionSourceConfig;

/**
 * The list of vision sources to pass to {@code RobotPose.initialize}. Each config is turned
 * into its source right away (the device is created when the config is added).
 *
 * <pre>
 * VisionConfig visionConfig = new VisionConfig()
 *     .addSource(BACK_2D_CONFIG)
 *     .addSource(QUEST_CONFIG);
 * </pre>
 */
public class VisionConfig {
    private final List<VisionSource> sources;

    /** An empty config; add sources with {@link #addSource}. */
    public VisionConfig() {
        this.sources = new ArrayList<>();
    }

    /** Creates a source for each config, in order. */
    public VisionConfig(BaseVisionSourceConfig... sourceConfigs) {
        this();

        for (BaseVisionSourceConfig sourceConfig : sourceConfigs) {
            sources.add(sourceConfig.getVisionSourceType().create(sourceConfig));
        }
    }

    /**
     * Creates the source for this config (its type comes from the config class) and adds it.
     *
     * @return this, for chaining.
     */
    public VisionConfig addSource(BaseVisionSourceConfig config) {
        sources.add(config.getVisionSourceType().create(config));
        return this;
    }

    /** @return Every source added so far (including the Quest, if there is one). */
    public List<VisionSource> getSources() {
        return sources;
    }
}