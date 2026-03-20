package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {

  @AutoLog
  public static class QuestNavIOInputs {
    public boolean connected = false;
    public boolean tracking = false;
    public int batteryPercent = 0;

    public Pose3d[] questPoses = new Pose3d[0];
    public double[] timestamps = new double[0];
  }

  public default void updateInputs(QuestNavIOInputs inputs) {}

  public default void setPose(Pose2d pose) {}
}
