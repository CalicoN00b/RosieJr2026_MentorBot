package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private final DCMotorSim shooterMotor;

  public ShooterIOSim() {
    shooterMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.connected = true;
    inputs.appliedVolts = shooterMotor.getInputVoltage();
    inputs.currentAmps = shooterMotor.getCurrentDrawAmps();
  }

  @Override
  public void setShooterDutyCycle(double output) {
    shooterMotor.setInputVoltage(output * 12);
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    shooterMotor.setAngularVelocity(velocityRadPerSec);
  }

  @Override
  public void setShooterNeutral() {
    shooterMotor.setInputVoltage(0);
  }
}
