package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Alert shooterDisconnectedAlert =
      new Alert("Shooter disconnected!", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    shooterDisconnectedAlert.set(!inputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void setDutyCycle(double output) {
    io.setShooterDutyCycle(output);
  }

  public void setVelocity(double velocityRadPerSec) {
    io.setShooterVelocity(velocityRadPerSec);
  }

  public void setNeutral() {
    io.setShooterNeutral();
  }
}
