package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class Intake {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
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
