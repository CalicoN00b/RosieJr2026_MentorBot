package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SuperstructureCommands {

  public static Command scoreFuelReal(
      Drive drive, Shooter shooter, CommandXboxController controller) {
    return Commands.parallel(new AimAtHub(drive, controller), ShooterCommands.runShooter(shooter));
  }

  public static Command scoreFuelSim(
      Drive drive, SwerveDriveSimulation driveSim, CommandXboxController controller) {

    return Commands.parallel(
        new AimAtHub(drive, controller),
        Commands.sequence(
                Commands.waitUntil(drive::aimedAtHub),
                ShooterCommands.visualizeFuel(driveSim),
                Commands.waitSeconds(0.18))
            .repeatedly());
  }
}
