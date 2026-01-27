package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean slapdownConnected = false;
    public Rotation2d slapdownPosition = Rotation2d.kZero;
    public double slapdownVelocityRadPerSec = 0.0;
    public double slapdownAppliedVolts = 0.0;
    public double slapdownCurrentAmps = 0.0;

    public boolean wheelsConnected = false;
    public double wheelsVelocityRadPerSec = 0.0;
    public double wheelsAppliedVolts = 0.0;
    public double wheelsCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setSlapdownPosition(Rotation2d rotation) {}

  public default void setWheelsOpenLoop(double output) {}

  public default void setWheelsVelocity(double velocityRadPerSec) {}
}
