package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterDutyCycle(double output) {}

  public default void setShooterVelocity(double velocityRadPerSec) {}

  public default void setShooterNeutral() {}
}
