package frc.robot.subsystems.hopper;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.function.DoubleSupplier;

public class HopperIOReal implements HopperIO {

  private SparkFlex motor;
  private RelativeEncoder encoder;

  private Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public HopperIOReal() {
    motor = new SparkFlex(0, MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    boolean sparkStickyFault = false;
    ifOk(
        motor,
        encoder::getVelocity,
        (value) -> inputs.velocityRadsPerSec = value * (2 * Math.PI) / 60);
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    ifOk(motor, motor::getMotorTemperature, (value) -> inputs.temperatureCelsius = value);
    connectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setOpenLoop(double output) {
    motor.set(output);
  }
}
