package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
