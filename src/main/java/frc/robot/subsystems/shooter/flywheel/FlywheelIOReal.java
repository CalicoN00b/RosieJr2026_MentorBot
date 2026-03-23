package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOReal implements FlywheelIO {

  private final TalonFX shooterMotor;

  StatusSignal<Voltage> motorVoltage;
  StatusSignal<Current> motorCurrent;
  StatusSignal<AngularVelocity> motorAngularVelocity;

  DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  VelocityVoltage velocityVoltageControl = new VelocityVoltage(null);
  NeutralOut neutralOutControl = new NeutralOut();

  public FlywheelIOReal() {
    shooterMotor = new TalonFX(0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = 1;
    config.Slot0.kP = 0.02;

    shooterMotor.getConfigurator().apply(config);

    motorVoltage = shooterMotor.getMotorVoltage();
    motorCurrent = shooterMotor.getSupplyCurrent();
    motorAngularVelocity = shooterMotor.getRotorVelocity();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected = shooterMotor.isConnected();
    inputs.appliedVolts = motorVoltage.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorAngularVelocity.getValueAsDouble());
  }

  @Override
  public void setShooterDutyCycle(double output) {
    shooterMotor.setControl(dutyCycleControl.withOutput(output));
  }

  @Override
  public void setShooterVelocity(double velocityRadPerSec) {
    shooterMotor.setControl(
        velocityVoltageControl.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setShooterNeutral() {
    shooterMotor.setControl(neutralOutControl);
  }
}
