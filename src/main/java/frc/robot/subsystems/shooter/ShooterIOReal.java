package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX shooterMotor;
  private final BangBangController bangBang = new BangBangController();

  public ShooterIOReal() {
    shooterMotor = new TalonFX(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = 0.75;

    shooterMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.connected = shooterMotor.isConnected();
    inputs.appliedVolts = shooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = shooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRadPerSec = shooterMotor.getRotorVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void setShooterDutyCycle(double output) {
    shooterMotor.setControl(new DutyCycleOut(output));
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    setShooterDutyCycle(
        bangBang.calculate(
            shooterMotor.getRotorVelocity().getValueAsDouble() * 2 * Math.PI, velocityRadPerSec));
  }

  @Override
  public void setShooterNeutral() {
    shooterMotor.setControl(new NeutralOut());
  }
}
