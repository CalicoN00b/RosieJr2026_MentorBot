package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;

public class QuestNavIOReal implements QuestNavIO {

  private final QuestNav questNav;
  private final Drive drive;

  public QuestNavIOReal(Drive drive) {
    questNav = new QuestNav();
    this.drive = drive;
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    questNav.commandPeriodic();

    inputs.connected = questNav.isConnected();
    inputs.tracking = questNav.isTracking();
    inputs.batteryPercent = questNav.getBatteryPercent().orElse(-1);

    List<Pose3d> questPoses = new LinkedList<Pose3d>();
    List<Pose2d> robotPoses = new LinkedList<Pose2d>();
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame poseFrame : poseFrames) {
      Pose3d questPose = poseFrame.questPose3d();
      questPoses.add(questPose);

      if (inputs.tracking) {
        Pose2d calculatedRobotPose =
            questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse()).toPose2d();
        robotPoses.add(calculatedRobotPose);
        double timestamp = poseFrame.dataTimestamp();

        drive.addVisionMeasurement(
            calculatedRobotPose, timestamp, QuestNavConstants.QUESTNAV_STD_DEVS);
      }
    }

    inputs.questPoses = questPoses.toArray(new Pose3d[0]);
    inputs.calculatedRobotPoses = robotPoses.toArray(new Pose2d[0]);
  }

  @Override
  public void setPose(Pose2d pose) {
    questNav.setPose(new Pose3d(pose).transformBy(QuestNavConstants.ROBOT_TO_QUEST));
  }
}
