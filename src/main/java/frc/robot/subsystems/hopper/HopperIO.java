package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public class HopperIOInputs {
    public boolean connected = false;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setOpenLoop(double output) {}
}
