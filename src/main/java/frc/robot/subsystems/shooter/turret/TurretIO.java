package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default Rotation2d getTurretAngle() {
    return Rotation2d.kZero;
  }

  public default void setMotorPosition(Angle position) {}

  public default void setTurretAngle(Rotation2d angle) {}

  public default void setOpenLoop(double output) {}

  public default void setNeutral() {}
}
