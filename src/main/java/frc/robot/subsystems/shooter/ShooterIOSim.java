package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final DCMotorSim sim;

  public ShooterIOSim() {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.0104726201, 0.75), gearbox);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sim.update(0.02);

    inputs.connected = true;
    inputs.appliedVolts = sim.getInputVoltage();
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setShooterDutyCycle(double output) {
    sim.setInputVoltage(output * 12);
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    sim.setAngularVelocity(velocityRadPerSec);
  }

  @Override
  public void setShooterNeutral() {
    sim.setInputVoltage(0);
  }
}
