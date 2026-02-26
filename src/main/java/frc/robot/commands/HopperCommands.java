package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;

public class HopperCommands {

  public static Command runHopper(Hopper hopper) {
    return Commands.runEnd(() -> hopper.setDutyCycle(1), () -> hopper.setDutyCycle(0), hopper);
  }
}
