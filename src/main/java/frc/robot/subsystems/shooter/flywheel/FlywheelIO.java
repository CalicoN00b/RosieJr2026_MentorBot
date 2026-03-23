package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setShooterDutyCycle(double output) {}

  public default void setShooterVelocity(double velocityRadPerSec) {}

  public default void setShooterNeutral() {}
}
