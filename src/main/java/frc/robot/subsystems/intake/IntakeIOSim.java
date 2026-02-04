package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim slapdownSim;
  private final DCMotorSim wheelsSim;

  private final DCMotor slapdownMotor = DCMotor.getKrakenX60(1);
  private final DCMotor wheelsMotor = DCMotor.getNeoVortex(1);

  public IntakeIOSim() {
    slapdownSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(slapdownMotor, 0.01, 2.5), slapdownMotor);
    wheelsSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(wheelsMotor, 0.01, 1), wheelsMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    slapdownSim.update(0.02);
    wheelsSim.update(0.02);

    inputs.slapdownConnected = true;
    inputs.slapdownPosition = Rotation2d.fromRadians(slapdownSim.getAngularPositionRad());
    inputs.slapdownVelocityRadPerSec = slapdownSim.getAngularVelocityRadPerSec();
    inputs.slapdownAppliedVolts = slapdownSim.getInputVoltage();
    inputs.slapdownCurrentAmps = slapdownSim.getCurrentDrawAmps();

    inputs.wheelsConnected = true;
    inputs.wheelsVelocityRadPerSec = wheelsSim.getAngularVelocityRadPerSec();
    inputs.wheelsAppliedVolts = wheelsSim.getInputVoltage();
    inputs.wheelsCurrentAmps = wheelsSim.getCurrentDrawAmps();
  }

  @Override
  public void setSlapdownPosition(Rotation2d rotation) {
    slapdownSim.setAngle(rotation.getRadians());
  }

  @Override
  public void setWheelsOpenLoop(double output) {
    wheelsSim.setInputVoltage(output * 12);
  }

  @Override
  public void setWheelsVelocity(double velocityRadPerSec) {
    wheelsSim.setAngularVelocity(velocityRadPerSec);
  }
}
