package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Everyone say, "Thank you, Red Rock!""
 *
 * <p>{@link
 * https://github.com/Red-Rock-Robotics-3006/FRC2026/blob/main/src/main/java/frc/robot/subsystems/shooter/autoaim/SOTMCalcs.java}
 */
public class SOTMCalculations {

  private static InterpolatingTreeMap<Double, Double> flightTimeMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  static {
    // Some of these are prob a bit wrong but whatever.
    flightTimeMap.put(1.0, 0.9);
    flightTimeMap.put(1.5, 1.0);
    flightTimeMap.put(2.0, 1.1);
    flightTimeMap.put(2.5, 1.2);
    flightTimeMap.put(3.0, 1.3);
    flightTimeMap.put(3.5, 1.4);
    flightTimeMap.put(4.0, 1.5);
    flightTimeMap.put(4.5, 1.6);
    flightTimeMap.put(5.0, 1.7);
    flightTimeMap.put(5.5, 1.8);
    flightTimeMap.put(6.0, 1.9);
  }

  /**
   * Uses the Secant Method to adjust a pose given field speeds, current position, and a goal
   * postiion
   *
   * @param fieldRelativeSpeeds - The field relative speeds of the turret
   * @param currentPose - The current pose of the turret
   * @param targetPose - The pose to aim for
   * @return the adjusted target pose.
   */
  public static Pose2d getSecantMethodAdjustedPose(
      ChassisSpeeds fieldRelativeSpeeds, Pose2d currentPose, Pose2d targetPose) {
    // Initial measurements (before projecitng into the future)
    double t0 = 0;
    Pose2d p0 = currentPose;

    // TODO: create flight time table using measurements from testing
    // t1 and p1 will be used to project into the future (aka how far will the robot have gone by
    // the time t1 has passed)
    double t1 = flightTimeMap.get(p0.minus(currentPose).getTranslation().getNorm());
    Pose2d p1 =
        p0.transformBy(
            new Transform2d(
                new Translation2d(
                    -fieldRelativeSpeeds.vxMetersPerSecond * t1,
                    -fieldRelativeSpeeds.vyMetersPerSecond * t1),
                new Rotation2d()));

    // Now we will repeat the projecting into the future, getting closer and closer to the "perfect"
    // answer, and we will return an acceptable answer
    for (int i = 0; i < 100; i++) {
      if (Math.abs(t1 - t0) < 1e-5) break;

      // Really gotta get that flight time table
      double ft0 = flightTimeMap.get(p0.minus(currentPose).getTranslation().getNorm());
      double ft1 = flightTimeMap.get(p1.minus(currentPose).getTranslation().getNorm());
      double newTOF = (t0 * (ft1 - t1) - t1 * (ft0 - t0)) / (ft1 - t1 - ft0 + t0);

      t0 = t1;
      t1 = newTOF;

      p0 = p1;
      p1 =
          targetPose.transformBy(
              new Transform2d(
                  new Translation2d(
                      -fieldRelativeSpeeds.vxMetersPerSecond * newTOF,
                      -fieldRelativeSpeeds.vyMetersPerSecond * newTOF),
                  new Rotation2d()));
    }

    return p1;
  }
}
