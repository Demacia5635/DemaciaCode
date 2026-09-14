package frc.demacia.RobotPose.Vision.VisionTypes;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.RobotPose.RobotPose;
import frc.demacia.RobotPose.Vision.BaseVisionSource;
import frc.demacia.RobotPose.Vision.TimestampedVisionMeasurement;
import frc.demacia.RobotPose.Vision.visionConfigs.QuestConfig;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Wraps a QuestNav-connected Meta Quest headset as a VisionSource, plus the two additional
 * methods (hasDrifted/updatePose) RobotPose's re-anchor logic depends on. Uses QuestNav's
 * real, confirmed API (https://questnav.gg) -- commandPeriodic(), isConnected(),
 * isTracking(), getAllUnreadPoseFrames(), setPose(Pose3d) -- verified against QuestNav's own
 * current documentation rather than assumed.
 *
 * shouldUpdate()/isConnected()/periodic()/hasDrifted()/updatePose() are fully implemented
 * below. getPoseEstimates() is intentionally left as a stub -- per instruction, the actual
 * frame-to-measurement math (turning drained PoseFrames into TimestampedVisionMeasurements,
 * including the robotToQuest offset correction and per-frame stdDevs) is the user's to
 * write, not this assistant's.
 */
public class Quest extends BaseVisionSource {
    private final QuestNav questNav;

    private Pose2d pose;
    
    private double timestampSeconds;
    private PoseFrame[] poseFrames;

    private boolean hasUpdatedQuestIntialPose;
    private boolean hasQuestDisconnected;

    /**
     * @param config Static configuration for this source. offset is robotToQuest -- the
     *               geometric transform from the robot's center to the Quest headset's
     *               mount position, used to convert between the Quest's own reported pose
     *               and the robot's pose. (Confirmed real usage pattern from QuestNav's own
     *               docs: robotPose = questPose.transformBy(robotToQuest.inverse()) and
     *               questPose = robotPose.transformBy(robotToQuest) for the reverse
     *               direction, used by updatePose() below.) sourceName is accepted as part
     *               of the shared VisionSourceConfig shape but is not otherwise used by
     *               this class -- QuestNav does not use a name-based lookup the way
     *               Limelight does.
     */
    public Quest(QuestConfig config) {
        super(config);
        hasUpdatedQuestIntialPose = false;
        hasQuestDisconnected = true;
        questNav = new QuestNav();
        questNav.commandPeriodic();
    }

    @Override
    protected void addLog() {
        super.addLog();
        SmartDashboard.putData("vision/" + getName() + "/Reset Quest Pose", new InstantCommand(()->setPose(new Pose2d())).ignoringDisable(true));
    }

    /**
     * Gated on QuestNav's own isConnected() -- a coarser check than per-frame tracking
     * (isTracking), matching the confirmed design: shouldUpdate() answers "is it even worth
     * asking this loop," with finer-grained per-frame tracking filtering happening
     * separately, inside getPoseEstimates().
     */
    @Override
    public boolean shouldUpdate() {
        return questNav.isConnected() && hasUpdatedQuestIntialPose;
    }

    /**
     *
     * @return A list of pose measurements from this loop's unread frames, filtered to only
     *         those that were actually tracking.
     */
    @Override
    public List<TimestampedVisionMeasurement> getPoseEstimates() {
        return List.of(new TimestampedVisionMeasurement(pose, timestampSeconds, std));
    }

    @Override
    public boolean isConnected() {
        return questNav.isConnected();
    }

    /** Required by QuestNav to process incoming/outgoing data -- must be called every loop. */
    @Override
    public void periodic() {
        questNav.commandPeriodic();

        poseFrames = questNav.getAllUnreadPoseFrames();

        if (poseFrames.length > 0 && poseFrames[poseFrames.length - 1].isTracking()) {
            pose = new Pose2d(poseFrames[poseFrames.length - 1].questPose3d()
                .transformBy(offset.inverse()).toPose2d().getTranslation(), 
                RobotPose.getInstance().getGyroAngle());
            
            timestampSeconds = poseFrames[poseFrames.length - 1].dataTimestamp();
        }

        if (!hasQuestDisconnected && !isConnected()) {
            hasQuestDisconnected = true;
        }
    }

    
    public boolean hasDrifted() {
        return hasQuestDisconnected && isConnected();
    }

    
    public void setPose(Pose2d currentFusedPose) {
        Pose3d robotPose3d = new Pose3d(currentFusedPose);
        Pose3d questPose = robotPose3d.transformBy(offset);
        questNav.setPose(questPose);

        hasUpdatedQuestIntialPose = false;
        hasQuestDisconnected = false;
    }
}