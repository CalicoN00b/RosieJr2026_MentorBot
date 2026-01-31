package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();
  public static Translation2d fieldCenter = new Translation2d(fieldLength / 2, fieldWidth / 2);

  /** Pose2d defining the center of the hub on the blue alliance */
  public static final Pose2d hubCenter =
      new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), Rotation2d.kZero);
}
