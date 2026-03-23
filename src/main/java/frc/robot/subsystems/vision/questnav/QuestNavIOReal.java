package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class QuestNavIOReal implements QuestNavIO {

  private final QuestNav questNav;
  private final Supplier<Pose2d> drivePoseSupplier;

  public QuestNavIOReal(Supplier<Pose2d> drivePoseSupplier) {
    questNav = new QuestNav();
    this.drivePoseSupplier = drivePoseSupplier;

    questNav.onConnected(() -> setPose(this.drivePoseSupplier.get()));
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    questNav.commandPeriodic();

    inputs.connected = questNav.isConnected();
    inputs.tracking = questNav.isTracking();
    inputs.batteryPercent = questNav.getBatteryPercent().orElse(0);

    List<Pose3d> questPoses = new LinkedList<Pose3d>();
    List<Double> timestamps = new LinkedList<Double>();
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    // Implementation for using each PoseFrame per loop
    for (PoseFrame poseFrame : poseFrames) {
      questPoses.add(poseFrame.questPose3d());
      timestamps.add(poseFrame.dataTimestamp());
    }

    // Implementation for only using the latest PoseFrame
    // if (poseFrames.length != 0) {
    //   PoseFrame latestPoseFrame = poseFrames[poseFrames.length - 1];
    //   questPoses.add(latestPoseFrame.questPose3d());
    //   timestamps.add(latestPoseFrame.dataTimestamp());
    // }

    inputs.questPoses = questPoses.toArray(new Pose3d[0]);

    inputs.timestamps = new double[timestamps.size()];
    for (int i = 0; i < inputs.timestamps.length; i++) {
      inputs.timestamps[i] = timestamps.get(i);
    }
  }

  @Override
  public void setPose(Pose2d pose) {
    questNav.setPose(new Pose3d(pose).transformBy(QuestNavConstants.ROBOT_TO_QUEST));
  }
}
