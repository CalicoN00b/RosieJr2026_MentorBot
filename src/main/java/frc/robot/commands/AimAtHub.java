package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AimAtHub extends Command {

  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber("AimAtHub/ThetaKp", 5.0);
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber("AimAtHub/ThetaKd", 0.4);

  private final Drive drive;
  private final CommandXboxController controller;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(thetaKp.get(), 0, thetaKd.get(), new Constraints(8.0, 20));

  private boolean isFlipped;

  public AimAtHub(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    addRequirements(drive);

    this.controller = controller;

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
    Pose2d hubCenter = FieldConstants.hubCenter;
    if (isFlipped) {
      hubCenter = hubCenter.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg);
    }

    Rotation2d currentToHubAngle =
        hubCenter.getTranslation().minus(currentPose.getTranslation()).getAngle();

    Translation2d driveVelocity =
        getLinearVelocityFromJoysticks(-controller.getLeftY(), -controller.getLeftX());
    double thetaVelocity =
        thetaController.calculate(drive.getRotation().getRadians(), currentToHubAngle.getRadians());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX()
                * (drive.getMaxLinearSpeedMetersPerSec() / 2)
                * (isFlipped ? -1 : 1),
            driveVelocity.getY()
                * (drive.getMaxLinearSpeedMetersPerSec() / 2)
                * (isFlipped ? -1 : 1),
            thetaVelocity,
            currentPose.getRotation()));

    Logger.recordOutput("AimAtHub/ThetaMeasured", currentPose.getRotation());
    Logger.recordOutput("AimAtHub/ThetaGoal", currentToHubAngle);
    Logger.recordOutput("AimAtHub/ThetaError", currentPose.getRotation().minus(currentToHubAngle));
  }

  private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }
}
