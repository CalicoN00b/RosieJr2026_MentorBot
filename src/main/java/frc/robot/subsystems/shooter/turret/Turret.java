package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  @AutoLogOutput(key="Turret/Setpoint")
  private double setpoint;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void setTurretAngle(double degrees) {
    io.setTurretAngle(Rotation2d.fromDegrees(degrees));
    setpoint = degrees;
  }

  @AutoLogOutput(key="Turret/AtSetpoint")
  public boolean atSetpoint() {
    double currentTurretAngle = io.getTurretAngle().getDegrees();

    double upperLimit = setpoint + 2;
    double lowerLimit = setpoint - 2;

    return lowerLimit <= currentTurretAngle && currentTurretAngle <= upperLimit;
  }

  public Command runTrackingCommand(DoubleSupplier angleToTargetDegrees) {
    return Commands.runEnd(
        () -> {
          setTurretAngle(angleToTargetDegrees.getAsDouble());
        },
        io::setNeutral,
        this);
  }
}
