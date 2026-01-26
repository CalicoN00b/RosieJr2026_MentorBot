package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

/** IO implementation for real QuestNav hardware */
public class VisionIOQuestNav implements VisionIO {
  private final QuestNav questNav;
  private final Transform3d robotToQuest;
  private final Supplier<Pose2d> robotPose;

  /**
   * Creates a new VisionIOQuestNav
   *
   * @param robotToQuest The 3D position of the Quest relative to the robot
   * @param robotPose A supplier of the current robot pose, to reset the QuestNav if it should get
   *     disconnected
   */
  public VisionIOQuestNav(Transform3d robotToQuest, Supplier<Pose2d> robotPose) {
    questNav = new QuestNav();
    this.robotToQuest = robotToQuest;
    this.robotPose = robotPose;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    questNav.commandPeriodic();
    inputs.connected = questNav.isConnected();

    // We don't use tags in this implementation, so we get to set these to the default
    inputs.tagIds = new int[0];
    inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

    List<PoseObservation> poseObservations = new LinkedList<PoseObservation>();
    for (var poseFrame : questNav.getAllUnreadPoseFrames()) {
      if (questNav.isTracking()) {
        Pose3d robotPose = poseFrame.questPose3d().transformBy(robotToQuest.inverse());

        poseObservations.add(
            new PoseObservation(
                poseFrame.dataTimestamp(), robotPose, 0, 0, 0, PoseObservationType.QUESTNAV));
      }
    }

    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }
  }
}
