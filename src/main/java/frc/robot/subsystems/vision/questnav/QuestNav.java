package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.photonvision.PhotonVision.VisionConsumer;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class QuestNav extends SubsystemBase {

  private final QuestNavIO io;
  private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

  private final VisionConsumer consumer;

  public QuestNav(VisionConsumer consumer, QuestNavIO io) {
    this.io = io;
    this.consumer = consumer;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("QuestNav", inputs);

    List<Pose2d> calculatedRobotPoses = new LinkedList<Pose2d>();

    for (int i = 0; i < inputs.questPoses.length; i++) {
      consumer.accept(
          inputs.questPoses[i].toPose2d(),
          inputs.timestamps[i],
          QuestNavConstants.QUESTNAV_STD_DEVS);

      calculatedRobotPoses.add(inputs.questPoses[i].toPose2d());
    }

    Logger.recordOutput("QuestNav/RobotPoses", calculatedRobotPoses.toArray(new Pose2d[0]));
  }

  public void setPose(Pose2d pose) {
    io.setPose(pose);
  }
}
