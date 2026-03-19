package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Hopper motor disconnected!", AlertType.kError);

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Hopper", inputs);
    io.updateInputs(inputs);

    disconnectedAlert.set(!inputs.connected && Constants.currentMode != Constants.Mode.SIM);
  }

  public void setDutyCycle(double output) {
    io.setOpenLoop(output);
  }

  public void stop() {
    io.setOpenLoop(0);
  }

  public Command runHopperDutyCycleCommand(double speed) {
    return Commands.runEnd(
      () -> setDutyCycle(speed), 
      this::stop, 
      this);
  }
}
