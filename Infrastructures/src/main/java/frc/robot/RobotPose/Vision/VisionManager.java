package frc.robot.RobotPose.Vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import frc.robot.RobotPose.Vision.VisionTypes.Quest;

/**
 * Owns construction/configuration of the robot's VisionSources (Limelight cameras, Quest,
 * and any future sources) and hands a finished, immutable set to RobotPose. RobotPose never
 * constructs or configures sources itself -- it only consumes what VisionManager produces.
 *
 * Configured via {@link Builder}: call .withSource(...) once per generic VisionSource (any
 * number of times) and, at most once, .withQuest(...) if this robot has a Quest -- Quest is
 * optional and its absence is structural (simply never call .withQuest()), not represented
 * by a null/empty placeholder that has to be separately checked at configuration time.
 *
 * Quest, when present, is stored BOTH inside the generic source list (so it still
 * participates in RobotPose's uniform periodic()/shouldUpdate()/getPoseEstimates() loop
 * like any other VisionSource) AND separately, named, via getQuest() -- so RobotPose can
 * find "the" Quest for its one accepted special case (drift re-anchoring) without scanning
 * or type-checking the generic list.
 *
 * Singleton, matching RobotPose's own static-factory pattern: private constructor, static
 * initialize(VisionManager)/getInstance(). Once built, a VisionManager instance is
 * immutable -- getSources() returns an unmodifiable view, and nothing about a built instance
 * can change afterward, even if the Builder that produced it is reused (see Builder).
 */
public final class VisionManager {

    private static VisionManager instance;

    private final List<VisionSource> sources;
    private final Quest quest;

    private VisionManager(List<VisionSource> sources, Quest quest) {
        this.sources = Collections.unmodifiableList(sources);
        this.quest = quest;
    }

    /** Returns the configured, immutable list of all registered VisionSources (including Quest, if present). */
    public List<VisionSource> getSources() {
        return sources;
    }

    /** Returns the registered Quest, if this robot has one. Empty if .withQuest() was never called. */
    public Optional<Quest> getQuest() {
        return Optional.ofNullable(quest);
    }

    /**
     * Installs the singleton instance. Meant to be called at most once; call this from
     * RobotPose.initialize(...), not directly from Robot.java, per the confirmed design
     * (RobotPose.initialize is the single entry point callers use). A second call silently
     * overwrites the previous instance -- the caller is trusted to only call this once, per
     * the user's own confirmed choice to guarantee correct call order themselves rather than
     * have this enforced here.
     */
    public static synchronized void initialize(VisionManager manager) {
        instance = manager;
    }

    /**
     * Returns the singleton instance, or null if initialize(...) has not been called yet.
     */
    public static synchronized VisionManager getInstance() {
        return instance;
    }

    /**
     * Builds a VisionManager. .withSource(...) is repeatable (call once per camera/generic
     * source); .withQuest(...) is optional and may be called at most once (a robot has at
     * most one Quest headset) -- a second call is silently ignored (first call wins), since
     * a robot only having one Quest is a physical constraint the caller is trusted to
     * respect, rather than something enforced here.
     *
     * A single Builder instance is meant to be used once, ending in .build(). Calling
     * .withSource(...)/.withQuest(...) again on a Builder AFTER .build() has already been
     * called does not affect the VisionManager instance already produced -- that instance
     * is immutable from the moment .build() returns, by construction (a fresh internal list
     * is copied into it), not merely by convention. Continuing to configure a builder after
     * build() has no defined use case and simply has no effect on anything already built.
     */
    public static final class Builder {
        private final List<VisionSource> sources = new ArrayList<>();
        private Quest quest;

        public Builder withSource(VisionSource source) {
            Objects.requireNonNull(source, "source must not be null");
            sources.add(source);
            return this;
        }

        /**
         * If called more than once on the same Builder, the first call wins -- subsequent
         * calls are silently ignored. A robot has at most one Quest.
         */
        public Builder withQuest(Quest quest) {
            Objects.requireNonNull(quest, "quest must not be null");
            if (this.quest == null) {
                this.quest = quest;
            }
            return this;
        }

        public VisionManager build() {
            List<VisionSource> sourcesCopy = new ArrayList<>(sources);
            if (quest != null && !sourcesCopy.contains(quest)) {
                sourcesCopy.add(quest);
            }
            return new VisionManager(sourcesCopy, quest);
        }
    }
}