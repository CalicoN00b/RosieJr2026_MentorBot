package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.turret.Turret;
import java.util.function.Supplier;

public class ShooterSuperstructure extends SubsystemBase {

  private final Flywheel flywheel;
  private final Turret turret;

  private final Supplier<Pose2d> driveBasePose;
  private final Supplier<ChassisSpeeds> driveBaseFieldRelativeSpeeds;

  private double targetDistance; // Distance from current robot pose to target
  private double
      turretAngleToTarget; // Angle to set the turret to (angle from current robot pose to target +
  // robot rotation)

  public ShooterSuperstructure(
      Flywheel flywheel,
      Turret turret,
      Supplier<Pose2d> driveBasePose,
      Supplier<ChassisSpeeds> driveBaseFieldRelativeSpeeds) {
    this.flywheel = flywheel;
    this.turret = turret;
    this.driveBasePose = driveBasePose;
    this.driveBaseFieldRelativeSpeeds = driveBaseFieldRelativeSpeeds;
  }

  @Override
  public void periodic() {
    boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Translation2d hubCenter =
        isFlipped
            ? FieldConstants.hubCenter.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
            : FieldConstants.hubCenter;

    Pose2d currentDrivePose = driveBasePose.get();
    ChassisSpeeds currentDriveFieldSpeeds = driveBaseFieldRelativeSpeeds.get();

    Translation2d adjustedTargetPose =
        SOTMCalculations.getSecantMethodAdjustedPoseTranslation(
            currentDriveFieldSpeeds, currentDrivePose.getTranslation(), hubCenter);

    targetDistance =
        Units.metersToFeet(currentDrivePose.getTranslation().getDistance(adjustedTargetPose));
    turretAngleToTarget =
        adjustedTargetPose
            .minus(currentDrivePose.getTranslation())
            .getAngle()
            .plus(currentDrivePose.getRotation())
            .minus(isFlipped ? Rotation2d.k180deg : Rotation2d.kZero)
            .getDegrees();
  }

  public Command runTrackingCommand() {
    return Commands.parallel(
      flywheel.runTrackingCommand(() -> targetDistance),
      turret.runTrackingCommand(() -> turretAngleToTarget)
    );
  }
}
