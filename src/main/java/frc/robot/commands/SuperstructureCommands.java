package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SuperstructureCommands {

  /**
   * Command for scoring fuel.
   *
   * <p>Continuously aims at the hub, then scores fuel only when within the aiming threshold and
   * when shooter is at setpoint
   *
   * @param drive - Drive subsystem.
   * @param shooter - Intake subsystem.
   * @param hopper - Hopper subsystem.
   * @param driveSim - Drive simulation. Only used in sim for visualzing fuel.
   * @param intakeSim - Intake simulation. Only used in sim for visualizing fuel.
   * @param controller - Drive controller.
   * @return The command sequence for scoring fuel
   */
  public static Command scoreFuel(
      Drive drive,
      Shooter shooter,
      Hopper hopper,
      SwerveDriveSimulation driveSim,
      IntakeSimulation intakeSim,
      CommandXboxController controller) {
    Command scoringCommand;

    if (Constants.currentMode == Constants.Mode.SIM) {
      scoringCommand =
          Commands.parallel(
              new AimAtHub(drive, controller),
              ShooterCommands.runShooter(shooter),
              HopperCommands.runHopper(hopper),
              Commands.sequence(
                      Commands.waitUntil(drive::aimedAtHub),
                      SimCommands.visualizeScoringFuel(drive, driveSim, intakeSim)
                          .onlyIf(() -> intakeSim.getGamePiecesAmount() > 0),
                      Commands.waitSeconds(0.08))
                  .repeatedly());
    } else {
      scoringCommand =
          Commands.parallel(
              new AimAtHub(drive, controller),
              ShooterCommands.runShooter(shooter),
              HopperCommands.runHopper(hopper));
    }

    return scoringCommand;
  }

  public static Command passFuel(
      Shooter shooter, Hopper hopper, SwerveDriveSimulation driveSim, IntakeSimulation intakeSim) {
    Command passingCommand;

    if (Constants.currentMode == Constants.Mode.SIM) {
      passingCommand =
          Commands.parallel(
              ShooterCommands.runShooter(shooter),
              HopperCommands.runHopper(hopper),
              Commands.sequence(
                      SimCommands.visualizePassingFuel(driveSim, intakeSim)
                          .onlyIf(() -> intakeSim.getGamePiecesAmount() > 0),
                      Commands.waitSeconds(0.08))
                  .repeatedly());
    } else {
      passingCommand =
          Commands.parallel(ShooterCommands.runShooter(shooter), HopperCommands.runHopper(hopper));
    }

    return passingCommand;
  }

  public static Command scoreFuelAuto(
      Drive drive,
      Shooter shooter,
      Hopper hopper,
      SwerveDriveSimulation driveSim,
      IntakeSimulation intakeSim) {
    Command scoringCommand;

    if (Constants.currentMode == Constants.Mode.SIM) {
      scoringCommand =
          Commands.parallel(
              new AimAtHubAuto(drive),
              ShooterCommands.runShooter(shooter),
              HopperCommands.runHopper(hopper),
              Commands.sequence(
                      Commands.waitUntil(drive::aimedAtHub),
                      SimCommands.visualizeScoringFuel(drive, driveSim, intakeSim)
                          .onlyIf(() -> intakeSim.getGamePiecesAmount() > 0),
                      Commands.waitSeconds(0.08))
                  .repeatedly());
    } else {
      scoringCommand =
          Commands.parallel(
              new AimAtHubAuto(drive),
              ShooterCommands.runShooter(shooter),
              HopperCommands.runHopper(hopper));
    }

    return scoringCommand;
  }

  public static Command intakeFuel(Intake intake, Hopper hopper, IntakeSimulation intakeSim) {
    Command intakeCommand;

    if (Constants.currentMode == Constants.Mode.SIM) {
      intakeCommand =
          Commands.parallel(
              IntakeCommands.runIntake(intake),
              HopperCommands.runHopper(hopper),
              SimCommands.runIntake(intakeSim));
    } else {
      intakeCommand =
          Commands.parallel(IntakeCommands.runIntake(intake), HopperCommands.runHopper(hopper));
    }

    return intakeCommand;
  }
}
