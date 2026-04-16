package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOReal implements TurretIO {

  private final TalonFX turretMotor = new TalonFX(0);

  private final StatusSignal<Voltage> turretMotorAppliedVolts = turretMotor.getMotorVoltage();
  private final StatusSignal<Current> turretMotorCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> turretMotorAngularVelocity =
      turretMotor.getVelocity();

  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private final NeutralOut neutralControl = new NeutralOut();

  private final DutyCycleEncoder tooth13Encoder = new DutyCycleEncoder(0);
  private final DutyCycleEncoder tooth17Encoder = new DutyCycleEncoder(1);

  public TurretIOReal() {
    calibrateTurret();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.connected = turretMotor.isConnected();
    inputs.appliedVolts = turretMotorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = turretMotorCurrent.getValueAsDouble();
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(turretMotorAngularVelocity.getValueAsDouble());
  }

  @Override
  public Rotation2d getTurretAngle() {
    return Rotation2d.fromDegrees(crtDegrees());
  }

  @Override
  public void setMotorPosition(Angle position) {
    turretMotor.setControl(positionControl.withPosition(position.in(Rotations)));
  }

  @Override
  public void setTurretAngle(Rotation2d angle) {
    setMotorPosition(Degrees.of(MathUtil.clamp(angle.getDegrees() * (200 / 7), -135, 135)));
  }

  @Override
  public void setOpenLoop(double output) {
    turretMotor.setControl(dutyCycleControl.withOutput(output));
  }

  @Override
  public void setNeutral() {
    turretMotor.setControl(neutralControl);
  }

  private double crtDegrees() {
    double throughBore13Value = tooth13Encoder.get();
    if (throughBore13Value < 0) throughBore13Value = 1 - throughBore13Value;

    double throughBore17Value = tooth17Encoder.get();
    if (throughBore17Value < 0) throughBore17Value = 1 - throughBore17Value;

    double tooth13 = throughBore13Value * 13;
    double tooth13Remainder = tooth13 - (int) tooth13;
    double tooth17 = throughBore17Value * 17;

    double absToothCount = solveCRT((int) tooth13, (int) tooth17) + tooth13Remainder;

    return (absToothCount / 80) * 360 - 135;
  }

  private int solveCRT(int tooth13, int tooth17) {
    int inverse = modInverse(tooth13, tooth17);
    int x = (tooth13 + 13 * (Math.floorMod((tooth17 - tooth13) * inverse, 17))) % 221;
    return x == 0 ? 221 : x;
  }

  private int modInverse(int a, int m) {
    if (m == 1) return 0;

    int m0 = m, x0 = 0, x1 = 1;
    while (a > 1) {
      int q = a / m;
      int t = m;
      m = a % m;
      a = t;
      t = x0;
      x0 = x1 - q * x0;
      x1 = t;
    }

    return x1 < 0 ? x1 + m0 : x1;
  }

  private void calibrateTurret() {
    double currentTurretAngle = crtDegrees();
    double turretDegreesToMotorRotations = Units.degreesToRotations(currentTurretAngle * (200 / 7));
    turretMotor.setPosition(turretDegreesToMotorRotations);
  }
}
