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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.AimAtHubAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SimCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.turret.*;
import frc.robot.subsystems.vision.photonvision.*;
import frc.robot.subsystems.vision.questnav.*;
import frc.robot.util.ChooserListener;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LocalADStarAK;
import java.util.Map;
import java.util.function.DoubleSupplier;
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
  private final Flywheel flywheel;
  private final Turret turret;
  private final Intake intake;
  private final Hopper hopper;
  private final PhotonVision photonVision;
  private final QuestNav questNav;

  private final ShooterSuperstructure shooterSuperstructure;

  private SwerveDriveSimulation driveSim = null;
  private IntakeSimulation intakeSim = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Overrides
  private boolean shootOverride = false; // If true, allows you to auto shoot when hub is inactive.

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
        flywheel = new Flywheel(new FlywheelIOReal());
        turret = new Turret(new TurretIOReal());
        intake = new Intake(new IntakeIOReal());
        hopper = new Hopper(new HopperIOReal());
        photonVision =
            new PhotonVision(
                drive::addVisionMeasurement,
                new PhotonVisionIOReal(
                    PhotonVisionConstants.camera0Name, PhotonVisionConstants.robotToCamera0),
                new PhotonVisionIOReal(
                    PhotonVisionConstants.camera1Name, PhotonVisionConstants.robotToCamera1));
        questNav = new QuestNav(drive::addVisionMeasurement, new QuestNavIOReal(drive::getPose));
        shooterSuperstructure = new ShooterSuperstructure(flywheel, turret, drive::getPose, drive::getFieldRelativeChassisSpeeds);
        break;

      case SIM:
        driveSim =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

        intakeSim =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel", driveSim, Inches.of(27), Inches.of(10), IntakeSide.FRONT, 40);

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
                drive::addVisionMeasurement,
                new PhotonVisionIOSim(
                    PhotonVisionConstants.camera0Name,
                    PhotonVisionConstants.robotToCamera0,
                    driveSim::getSimulatedDriveTrainPose),
                new PhotonVisionIOSim(
                    PhotonVisionConstants.camera1Name,
                    PhotonVisionConstants.robotToCamera1,
                    driveSim::getSimulatedDriveTrainPose));
        flywheel = new Flywheel(new FlywheelIOSim() {});
        turret = new Turret(new TurretIO() {});
        intake = new Intake(new IntakeIOSim());
        hopper = new Hopper(new HopperIO() {});
        questNav = new QuestNav(drive::addVisionMeasurement, new QuestNavIO() {});
        shooterSuperstructure = new ShooterSuperstructure(flywheel, turret, driveSim::getSimulatedDriveTrainPose, driveSim::getDriveTrainSimulatedChassisSpeedsFieldRelative);
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
        photonVision = new PhotonVision(drive::addVisionMeasurement, new PhotonVisionIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        turret = new Turret(new TurretIO() {});
        intake = new Intake(new IntakeIO() {});
        hopper = new Hopper(new HopperIO() {});
        questNav = new QuestNav(drive::addVisionMeasurement, new QuestNavIO() {});
        shooterSuperstructure = new ShooterSuperstructure(flywheel, turret, drive::getPose, drive::getFieldRelativeChassisSpeeds);
        break;
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        drive::getPose,
        (pose) -> {
          drive.setPose(pose);
          questNav.setPose(pose);
        },
        drive::getRobotRelativeChassisSpeeds,
        drive::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        DriveConstants.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);

    // Set pathfinder and logging path for PathPlanner
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Drive/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Drive/TrajectorySetpoint", targetPose);
        });

    // Register NamedCommands for PathPlanner
    NamedCommands.registerCommands(
        Map.of(
            "Intake",
                Commands.parallel(
                        intake.runWheelsDutyCycleCommand(1), hopper.runHopperDutyCycleCommand(0.7))
                    .alongWith(
                        SimCommands.runIntake(intakeSim)
                            .onlyIf(() -> Constants.currentMode == Constants.Mode.SIM)),
            "Shoot",
                Commands.parallel(
                        shooterSuperstructure.runTrackingCommand())
                    .onlyWhile(() -> !(turret.atSetpoint() && flywheel.atSetpoint()))
                    .andThen(
                        Commands.parallel(
                            shooterSuperstructure.runTrackingCommand(),
                            hopper.runHopperDutyCycleCommand(0.7),
                            SimCommands.visualizeScoringFuelTurretSOTM(driveSim, intakeSim)
                                .onlyIf(() -> Constants.currentMode == Constants.Mode.SIM)))));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.onChange(
        new ChooserListener(autoChooser.getSendableChooser()::getSelected, drive, questNav));

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
    DoubleSupplier driveX = () -> -controller.getLeftX();
    DoubleSupplier driveY = () -> -controller.getLeftY();
    DoubleSupplier driveOmega = () -> -controller.getRightX();
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driveY, driveX, driveOmega));

    // Reset the robot to face forward (front of the robot facing away from alliance wall) when
    // presed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Translation2d driveTranslation = drive.getPose().getTranslation();
                      boolean isFlipped =
                          DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                      Pose2d poseAfterReset =
                          new Pose2d(
                              driveTranslation, isFlipped ? Rotation2d.k180deg : Rotation2d.kZero);
                      drive.setPose(poseAfterReset);
                      questNav.setPose(poseAfterReset);
                    },
                    drive,
                    questNav)
                .ignoringDisable(true));

    // Sequence for shooting at the hub.
    // Auto aims and warms up the shooter
    // Then spins up the hopper to feed the shooter
    controller
        .rightTrigger()
        .and(() -> HubShiftUtil.isHubActive())
        .or(() -> shootOverride)
        .whileTrue(DriveCommands.joystickDriveSOTM(drive, driveX, driveY, driveOmega))
        .whileTrue(shooterSuperstructure.runTrackingCommand())
        .and(turret::atSetpoint)
        .and(flywheel::atSetpoint)
        .whileTrue(hopper.runHopperDutyCycleCommand(0.7))
        .and(() -> Constants.currentMode == Constants.Mode.SIM)
        .whileTrue(SimCommands.visualizeScoringFuelTurretSOTM(driveSim, intakeSim));

    controller
        .rightTrigger()
        .and(() -> !HubShiftUtil.isHubActive())
        .and(() -> !shootOverride) // Don't rumble if shoot override is on
        .onTrue(
            Commands.runEnd(
                    () -> controller.setRumble(RumbleType.kBothRumble, 1),
                    () -> controller.setRumble(RumbleType.kBothRumble, 0))
                .withTimeout(0.5));

    // Sequence for passing fuel
    controller
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                flywheel.runFixedVelocityCommand(210), hopper.runHopperDutyCycleCommand(0.7)))
        .and(() -> Constants.currentMode == Constants.Mode.SIM)
        .whileTrue(SimCommands.visualizePassingFuel(driveSim, intakeSim));

    // Sequence for intaking fuel
    controller
        .rightBumper()
        .whileTrue(intake.runWheelsDutyCycleCommand(1))
        .whileTrue(hopper.runHopperDutyCycleCommand(0.7))
        .and(() -> Constants.currentMode == Constants.Mode.SIM)
        .whileTrue(SimCommands.runIntake(intakeSim));

    controller
        .povUp()
        .onTrue(Commands.runOnce(() -> shootOverride = true).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> shootOverride = false).ignoringDisable(true));
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
    Logger.recordOutput("FieldSimulation/RobotPose", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput("FieldSimulation/FuelInIntake", intakeSim.getGamePiecesAmount());
  }

  public void updateElastic() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      SmartDashboard.putNumber(
          "(SIM) Points Scored",
          SimulatedArena.getInstance().getScore(DriverStation.getAlliance().orElse(Alliance.Blue)));
    }
    SmartDashboard.putBoolean("Hub Active", HubShiftUtil.isHubActive());
    SmartDashboard.putBoolean("Shoot Override", shootOverride);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }
}
