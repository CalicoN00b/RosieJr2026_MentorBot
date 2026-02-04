package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class SimCommands {

  public static Command visualizeFuel(SwerveDriveSimulation driveSim) {
    return Commands.runOnce(
        () ->
            SimulatedArena.getInstance()
                .addGamePieceProjectile(
                    new RebuiltFuelOnFly(
                        driveSim.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(),
                        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        driveSim.getSimulatedDriveTrainPose().getRotation(),
                        Meters.of(0.1),
                        MetersPerSecond.of(9.4488),
                        Degrees.of(75))));
  }

  public static Command runIntake(IntakeSimulation intakeSim) {
    return Commands.startEnd(intakeSim::startIntake, intakeSim::stopIntake);
  }
}
