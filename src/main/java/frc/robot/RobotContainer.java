// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.photonvision.*;
import frc.robot.subsystems.vision.questnav.*;
import java.util.Map;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Hopper hopper;
  private final PhotonVision photonVision;
  private final QuestNav questNav;

  private SwerveDriveSimulation driveSim = null;
  private IntakeSimulation intakeSim = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOReal(),
                new ModuleIOReal(0),
                new ModuleIOReal(1),
                new ModuleIOReal(2),
                new ModuleIOReal(3),
                (pose) -> {});
        shooter = new Shooter(new ShooterIOReal());
        intake = new Intake(new IntakeIOReal());
        hopper = new Hopper(new HopperIOReal());
        photonVision =
            new PhotonVision(
                drive,
                new PhotonVisionIOReal(
                    PhotonVisionConstants.camera0Name, PhotonVisionConstants.robotToCamera0),
                new PhotonVisionIOReal(
                    PhotonVisionConstants.camera1Name, PhotonVisionConstants.robotToCamera1));
        questNav = new QuestNav(new QuestNavIOReal(drive));
        break;

      case SIM:
        driveSim =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

        intakeSim =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel", driveSim, Inches.of(20.125), Inches.of(10), IntakeSide.FRONT, 40);

        // Add preload to sim
        intakeSim.setGamePiecesCount(8);

        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSim.getGyroSimulation()) {},
                new ModuleIOSim(driveSim.getModules()[0]),
                new ModuleIOSim(driveSim.getModules()[1]),
                new ModuleIOSim(driveSim.getModules()[2]),
                new ModuleIOSim(driveSim.getModules()[3]),
                driveSim::setSimulationWorldPose);
        photonVision =
            new PhotonVision(
                drive,
                new PhotonVisionIOSim(
                    PhotonVisionConstants.camera0Name,
                    PhotonVisionConstants.robotToCamera0,
                    driveSim::getSimulatedDriveTrainPose),
                new PhotonVisionIOSim(
                    PhotonVisionConstants.camera1Name,
                    PhotonVisionConstants.robotToCamera1,
                    driveSim::getSimulatedDriveTrainPose));
        shooter = new Shooter(new ShooterIOSim() {});
        intake = new Intake(new IntakeIOSim());
        hopper = new Hopper(new HopperIO() {});
        questNav = new QuestNav(new QuestNavIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        photonVision = new PhotonVision(drive, new PhotonVisionIO() {});
        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        hopper = new Hopper(new HopperIO() {});
        questNav = new QuestNav(new QuestNavIO() {});
        break;
    }

    NamedCommands.registerCommands(
        Map.of(
            "Intake", SuperstructureCommands.intakeFuel(intake, hopper, intakeSim),
            "Shoot",
                SuperstructureCommands.scoreFuelAuto(drive, shooter, hopper, driveSim, intakeSim)));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reset gyro to 0° when start button is pressed
    final Runnable resetGryo =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSim.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
    controller.start().onTrue(Commands.runOnce(resetGryo, drive).ignoringDisable(true));

    controller
        .rightBumper()
        .whileTrue(SuperstructureCommands.intakeFuel(intake, hopper, intakeSim));

    controller.x().whileTrue(SuperstructureCommands.passFuel(shooter, hopper, driveSim, intakeSim));

    controller.y().whileTrue(new AimAtHub(drive, controller));

    controller
        .rightTrigger()
        .whileTrue(
            SuperstructureCommands.scoreFuel(
                drive, shooter, hopper, driveSim, intakeSim, controller));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput("FieldSimulation/FuelInIntake", intakeSim.getGamePiecesAmount());
  }
}
