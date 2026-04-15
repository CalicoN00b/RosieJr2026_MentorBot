package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @AutoLogOutput(key = "Flywheel/Setpoint")
  private double setpoint = 0;

  private final Alert shooterDisconnectedAlert =
      new Alert("Flywheel motor disconnected!", AlertType.kError);

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    shooterDisconnectedAlert.set(!inputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void setDutyCycle(double output) {
    io.setShooterDutyCycle(output);
  }

  public void setVelocity(double velocityRadPerSec) {
    setpoint = velocityRadPerSec;
    io.setShooterVelocity(velocityRadPerSec);
  }

  public void setNeutral() {
    setpoint = 0;
    io.setShooterNeutral();
  }

  @AutoLogOutput(key = "Flywheel/AtSetpoint")
  public boolean atSetpoint() {
    if (Constants.currentMode == Constants.Mode.SIM || setpoint == 0) return true;

    double upperLimit = setpoint * 1.02;
    double lowerLimit = setpoint * 0.98;

    return lowerLimit <= inputs.velocityRadPerSec && inputs.velocityRadPerSec <= upperLimit;
  }

  @AutoLogOutput(key = "Flywheel/CalculatedVelocity")
  public double calculateShooterVelocity(double distanceToHub) {
    return (21 + 15 * (distanceToHub - 4.1) / 13.2)
        * 1.047; // * 5.75 * 2. May also need rotationsToRadians
  }

  public Command runDutyCycleCommand(double speed) {
    return Commands.runEnd(() -> setDutyCycle(speed), this::setNeutral, this);
  }

  public Command runFixedVelocityCommand(double velocityRadsPerSec) {
    return Commands.runEnd(() -> setVelocity(velocityRadsPerSec), this::setNeutral, this);
  }

  public Command runTrackingCommand(DoubleSupplier distanceToHub) {
    return Commands.runEnd(
        () -> {
          setVelocity(calculateShooterVelocity(distanceToHub.getAsDouble()));
        },
        this::setNeutral,
        this);
  }
}
