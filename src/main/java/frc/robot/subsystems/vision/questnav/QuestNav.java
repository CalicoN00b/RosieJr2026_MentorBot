package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class QuestNav extends SubsystemBase {

  private final QuestNavIO io;
  private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

  public QuestNav(QuestNavIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("QuestNav", inputs);
  }

  public void setPose(Pose2d pose) {
    io.setPose(pose);
  }
}
