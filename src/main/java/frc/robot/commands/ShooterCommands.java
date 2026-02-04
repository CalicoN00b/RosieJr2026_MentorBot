package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterCommands {

  private ShooterCommands() {}

  public static Command runShooter(Shooter shooter) {
    return Commands.runEnd(() -> shooter.setDutyCycle(0.8), () -> shooter.setNeutral(), shooter);
  }

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
}
