package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

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
