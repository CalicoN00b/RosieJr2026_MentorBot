package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.SOTMCalculations;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class SimCommands {

  public static Command visualizeScoringFuel(
      Drive drive, SwerveDriveSimulation driveSim, IntakeSimulation intakeSim) {
    return Commands.repeatingSequence(
        Commands.runOnce(
                () -> {
                  double launchVelocity =
                      (21 + 15 * ((drive.distanceFromHubFeet() - 4.1) / 13.2)) * 1.047;

                  SimulatedArena.getInstance()
                      .addGamePieceProjectile(
                          new RebuiltFuelOnFly(
                              driveSim.getSimulatedDriveTrainPose().getTranslation(),
                              new Translation2d(),
                              driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                              driveSim.getSimulatedDriveTrainPose().getRotation(),
                              Meters.of(0.1),
                              FeetPerSecond.of(launchVelocity),
                              Degrees.of(75)));
                  intakeSim.obtainGamePieceFromIntake();
                })
            .onlyIf(() -> intakeSim.getGamePiecesAmount() > 0),
        Commands.waitSeconds(0.08));
  }

  public static Command visualizeScoringFuelTurret(
      Drive drive, SwerveDriveSimulation driveSim, IntakeSimulation intakeSim) {
    return Commands.repeatingSequence(
        Commands.runOnce(
                () -> {
                  double launchVelocity =
                      (21 + 15 * ((drive.distanceFromHubFeet() - 4.1) / 13.2)) * 1.047;

                  SimulatedArena.getInstance()
                      .addGamePieceProjectile(
                          new RebuiltFuelOnFly(
                              driveSim.getSimulatedDriveTrainPose().getTranslation(),
                              new Translation2d(),
                              driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                              drive.angleToHub(),
                              Meters.of(0.1),
                              FeetPerSecond.of(launchVelocity),
                              Degrees.of(75)));
                  intakeSim.obtainGamePieceFromIntake();
                })
            .onlyIf(() -> intakeSim.getGamePiecesAmount() > 0),
        Commands.waitSeconds(0.08));
  }

  public static Command visualizeScoringFuelTurretSOTF(
      Drive drive, SwerveDriveSimulation driveSim, IntakeSimulation intakeSim) {
    return Commands.repeatingSequence(
        Commands.runOnce(
                () -> {
                  boolean isFlipped =
                      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                  Translation2d hubCenter =
                      isFlipped
                          ? FieldConstants.hubCenter.rotateAround(
                              FieldConstants.fieldCenter, Rotation2d.k180deg)
                          : FieldConstants.hubCenter;

                  Pose2d currentPose = drive.getPose();
                  ChassisSpeeds fieldSpeeds = drive.getFieldRelativeChassisSpeeds();

                  Pose2d adjustedTargetPose =
                      SOTMCalculations.getSecantMethodAdjustedPose(
                          fieldSpeeds, currentPose, new Pose2d(hubCenter, Rotation2d.kZero));

                  double distToAdjustedPose =
                      Units.metersToFeet(
                          currentPose
                              .getTranslation()
                              .getDistance(adjustedTargetPose.getTranslation()));
                  Rotation2d angleToAdjustedPose =
                      adjustedTargetPose
                          .getTranslation()
                          .minus(currentPose.getTranslation())
                          .getAngle();

                  double launchVelocity = (21 + 15 * ((distToAdjustedPose - 4.1) / 13.2)) * 1.047;

                  Logger.recordOutput("SOTM/AdjustedTargetPose", adjustedTargetPose);
                  Logger.recordOutput("SOTM/DistToAdjustedPoseFeet", distToAdjustedPose);
                  Logger.recordOutput(
                      "SOTM/AngleToAdjustedPoseDegrees", angleToAdjustedPose.getDegrees());
                  Logger.recordOutput("SOTM/LaunchVelocityFeetPerSec", launchVelocity);

                  SimulatedArena.getInstance()
                      .addGamePieceProjectile(
                          new RebuiltFuelOnFly(
                              driveSim.getSimulatedDriveTrainPose().getTranslation(),
                              new Translation2d(),
                              driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                              angleToAdjustedPose,
                              Meters.of(0.1),
                              FeetPerSecond.of(launchVelocity),
                              Degrees.of(75)));
                  intakeSim.obtainGamePieceFromIntake();
                })
            .onlyIf(() -> intakeSim.getGamePiecesAmount() > 0),
        Commands.waitSeconds(0.08));
  }

  public static Command visualizePassingFuel(
      SwerveDriveSimulation driveSim, IntakeSimulation intakeSim) {
    return Commands.repeatingSequence(
        Commands.runOnce(
                () -> {
                  SimulatedArena.getInstance()
                      .addGamePieceProjectile(
                          new RebuiltFuelOnFly(
                              driveSim.getSimulatedDriveTrainPose().getTranslation(),
                              new Translation2d(),
                              driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                              driveSim.getSimulatedDriveTrainPose().getRotation(),
                              Meters.of(0.1),
                              FeetPerSecond.of(20),
                              Degrees.of(75)));
                  intakeSim.obtainGamePieceFromIntake();
                })
            .onlyIf(() -> intakeSim.getGamePiecesAmount() > 0),
        Commands.waitSeconds(0.08));
  }

  public static Command runIntake(IntakeSimulation intakeSim) {
    return Commands.startEnd(intakeSim::startIntake, intakeSim::stopIntake);
  }
}
