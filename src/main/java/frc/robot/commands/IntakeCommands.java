package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {

  public static Command runIntake(Intake intake) {
    return Commands.startEnd(
        () -> intake.setWheelsDutyCycle(1), () -> intake.setWheelsDutyCycle(0), intake);
  }
}
