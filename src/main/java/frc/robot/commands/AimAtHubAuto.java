package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AimAtHubAuto extends Command {

  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber("AimAtHubAuto/ThetaKp", 8.0);
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber("AimAtHubAuto/ThetaKd", 0.4);

  private final Drive drive;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(thetaKp.get(), 0, thetaKd.get(), new Constraints(8.0, 20));

  private boolean isFlipped;

  public AimAtHubAuto(Drive drive) {
    this.drive = drive;
    addRequirements(drive);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    thetaController.reset(drive.getRotation().getRadians());
    isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  @Override
  public void execute() {
    if (thetaKp.hasChanged(hashCode()) || thetaKp.hasChanged(hashCode())) {
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
    }

    Pose2d currentPose = drive.getPose();
    Translation2d hubCenter = FieldConstants.hubCenter;
    if (isFlipped) {
      hubCenter = hubCenter.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg);
    }

    Rotation2d currentToHubAngle = hubCenter.minus(currentPose.getTranslation()).getAngle();

    double thetaVelocity =
        thetaController.calculate(drive.getRotation().getRadians(), currentToHubAngle.getRadians());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, thetaVelocity, currentPose.getRotation()));

    Logger.recordOutput("AimAtHubAuto/ThetaMeasured", currentPose.getRotation());
    Logger.recordOutput("AimAtHubAuto/ThetaGoal", currentToHubAngle);
    Logger.recordOutput(
        "AimAtHubAuto/ThetaError", currentPose.getRotation().minus(currentToHubAngle));
  }
}
