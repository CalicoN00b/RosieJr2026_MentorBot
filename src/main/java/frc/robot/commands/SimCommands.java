package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

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
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

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
                  Translation2d targetPose = hubCenter.minus(drive.getPose().getTranslation());
                  double distance = targetPose.getNorm();

                  double launchVelocity =
                      (21 + 15 * ((Units.metersToFeet(distance) - 4.1) / 13.2)) * 1.047;
                  Translation2d targetVector =
                      targetPose.div(distance).times(Units.feetToMeters(launchVelocity));

                  ChassisSpeeds fieldRelativeSpeeds =
                      ChassisSpeeds.fromRobotRelativeSpeeds(
                          drive.getChassisSpeeds(), drive.getRotation());
                  Translation2d robotVelocity =
                      new Translation2d(
                          fieldRelativeSpeeds.vxMetersPerSecond,
                          fieldRelativeSpeeds.vyMetersPerSecond);

                  Translation2d shotVector = targetVector.minus(robotVelocity);

                  SimulatedArena.getInstance()
                      .addGamePieceProjectile(
                          new RebuiltFuelOnFly(
                              driveSim.getSimulatedDriveTrainPose().getTranslation(),
                              new Translation2d(),
                              driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                              shotVector.getAngle(),
                              Meters.of(0.1),
                              MetersPerSecond.of(shotVector.getNorm()),
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
