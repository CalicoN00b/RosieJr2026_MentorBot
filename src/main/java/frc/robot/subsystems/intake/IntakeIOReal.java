package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class IntakeIOReal implements IntakeIO {

  private final TalonFX slapdownMotor;
  private final SparkFlex wheelMotor;
  private final RelativeEncoder wheelEncoder;

  private final Debouncer wheelsConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public IntakeIOReal() {
    slapdownMotor = new TalonFX(0);
    wheelMotor = new SparkFlex(0, MotorType.kBrushless);
    wheelEncoder = wheelMotor.getEncoder();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.slapdownConnected = slapdownMotor.isConnected();
    inputs.slapdownPosition =
        Rotation2d.fromRotations(slapdownMotor.getPosition().getValueAsDouble());
    inputs.slapdownVelocityRadPerSec = slapdownMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
    inputs.slapdownAppliedVolts = slapdownMotor.getMotorVoltage().getValueAsDouble();
    inputs.slapdownCurrentAmps = slapdownMotor.getSupplyCurrent().getValueAsDouble();

    sparkStickyFault = false;
    ifOk(
        wheelMotor,
        wheelEncoder::getVelocity,
        (value) -> inputs.wheelsVelocityRadPerSec = value * (2 * Math.PI) / 60);
    ifOk(
        wheelMotor,
        new DoubleSupplier[] {wheelMotor::getAppliedOutput, wheelMotor::getBusVoltage},
        (values) -> inputs.wheelsAppliedVolts = values[0] * values[1]);
    ifOk(wheelMotor, wheelMotor::getOutputCurrent, (value) -> inputs.wheelsCurrentAmps = value);
    inputs.wheelsConnected = wheelsConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setSlapdownPosition(Rotation2d rotation) {
    slapdownMotor.setControl(new PositionDutyCycle(rotation.getMeasure()));
  }

  @Override
  public void setWheelsOpenLoop(double output) {
    wheelMotor.set(output);
  }

  @Override
  public void setWheelsVelocity(double velocityRadPerSec) {
    // TODO: Figure out how to convert from rad per sec to duty cycle
  }
}
