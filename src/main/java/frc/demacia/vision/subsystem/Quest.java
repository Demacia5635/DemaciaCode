package frc.demacia.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Quest extends SubsystemBase {

  private Transform3d questOffset;
  private double timestamp;
  private QuestNav questNav;
  private Field2d robotField;
  private Pose3d QuestPose;
  private PoseFrame[] poseFrames;

  public Quest(Transform3d questOffset) {
    this.questOffset = questOffset;
    timestamp = 0;
    questNav = new QuestNav();
    questNav.commandPeriodic();
    robotField = new Field2d();
    QuestPose = new Pose3d();

    addLog();
  }
  
  @SuppressWarnings("unchecked")
  private void addLog() {
    LogManager.addEntry("Quest/X", () -> getRobotPose2d().getX()).build();
    LogManager.addEntry("Quest/Y", () -> getRobotPose2d().getY()).build();
    LogManager.addEntry("Quest/is working", () -> isWorking()).build();
    LogManager.addEntry("Quest/Latency", questNav::getLatency).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
    LogManager.addEntry("Quest/Battery", questNav::getBatteryPercent).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
    LogManager.addEntry("Quest/LibVersion", questNav::getLibVersion).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();

    SmartDashboard.putData("Quest/Reset Quest Pose", new InstantCommand(()->setQuestPose(new Pose3d())).ignoringDisable(true));
    SmartDashboard.putData("Quest/robotField", robotField);
  }


  // Set robot pose (transforms to Quest frame and sends to QuestNav)
  public void setQuestPose(Pose3d currentBotpose) {
    questNav.setPose(currentBotpose.transformBy(questOffset));// the transformBy is to switch x & y and gives back
   }                                                               // the hight of the quest

  /**
   * * @return the center of the robot form quest
   */
  public Pose2d getRobotPose2d() {
    return QuestPose.transformBy(questOffset.inverse()).toPose2d();
  }


  // Check if Quest is connected
  public boolean isConnected() {
    return questNav.isConnected();
  }

  public boolean isWorking() {
    return isConnected() && isTracking() && questNav.getBatteryPercent().getAsInt() > 10;
  }


  // Check if Quest is tracking
  public boolean isTracking() {
    return questNav.isTracking();
  }

  // gives me the timestamp of the newst frame
  public double getTimestamp() {
    return timestamp;
  }
  
  public void setHeading(Rotation2d angle){
    setQuestPose(new Pose3d(getRobotPose2d().getX(),getRobotPose2d().getY(),QuestPose.getZ(),new Rotation3d(angle)));
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0 && poseFrames[poseFrames.length - 1].isTracking()) {
      QuestPose = poseFrames[poseFrames.length - 1].questPose3d();
      timestamp = poseFrames[poseFrames.length - 1].dataTimestamp();

      robotField.setRobotPose(QuestPose.transformBy(questOffset.inverse()).toPose2d());
    }
  }
}