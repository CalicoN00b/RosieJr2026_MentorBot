package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

  private final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private final DCMotorSim sim;

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;

  public ShooterIOSim() {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.0141593652, 0.75), gearbox);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    appliedVolts = gearbox.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.appliedVolts = appliedVolts;
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
