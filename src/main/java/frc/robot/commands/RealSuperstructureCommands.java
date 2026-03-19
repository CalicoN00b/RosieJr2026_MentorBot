package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class RealSuperstructureCommands {

  public static Command scoreFuel(
      Drive drive, Shooter shooter, Hopper hopper, CommandXboxController controller) {

    DoubleSupplier shooterVelocity =
        () ->
            (21
                + 15
                    * (drive.distanceFromHubFeet() - 4.1)
                    / 13.2); // * 1.047 * 5.75 * 2. May also need rotationsToRadians

    return Commands.parallel(
            new AimAtHub(drive, controller),
            ShooterCommands.runShooterVelocity(shooter, shooterVelocity))
        .onlyWhile(() -> !(drive.aimedAtHub() || shooter.atSetpoint()))
        .andThen(
            Commands.parallel(
                new AimAtHub(drive, controller),
                ShooterCommands.runShooterVelocity(shooter, shooterVelocity),
                HopperCommands.runHopper(hopper)));
  }

  public static Command scoreFuelAuto(Drive drive, Shooter shooter, Hopper hopper) {

    DoubleSupplier shooterVelocity =
        () ->
            (21
                + 15
                    * (drive.distanceFromHubFeet() - 4.1)
                    / 13.2); // * 1.047 * 5.75 * 2. May also need rotationsToRadians

    return Commands.parallel(
            new AimAtHubAuto(drive), ShooterCommands.runShooterVelocity(shooter, shooterVelocity))
        .onlyWhile(() -> !(drive.aimedAtHub() || shooter.atSetpoint()))
        .andThen(
            Commands.parallel(
                new AimAtHubAuto(drive),
                ShooterCommands.runShooterVelocity(shooter, shooterVelocity),
                HopperCommands.runHopper(hopper)));
  }
}
