package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert slapdownDisconnectedAlert = new Alert("Intake slapdown motor disconnected!", AlertType.kError);
  private final Alert wheelsDisconnected = new Alert("Intake wheels motor disconnected", AlertType.kError);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
      Logger.processInputs("Intake", inputs);
      io.updateInputs(inputs);

      slapdownDisconnectedAlert.set(!inputs.slapdownConnected && Constants.currentMode != Mode.SIM);
      wheelsDisconnected.set(!inputs.wheelsConnected && Constants.currentMode != Mode.SIM);
  }

  public void setSlapdownPosition(Rotation2d angle) {
    io.setSlapdownPosition(angle);
  }

  public void setWheelsDutyCycle(double output) {
    io.setWheelsOpenLoop(output);
  }

  public void setWheelsVelocity(double velocityRadPerSec) {
    io.setWheelsVelocity(velocityRadPerSec);
  }
}
