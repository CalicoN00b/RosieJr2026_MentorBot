package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.Flywheel;

public class ShooterSuperstructure extends SubsystemBase {

  // Will also initialize turret and hood here
  private final Flywheel flywheel;

  // Will also need to take in turret and hood once those have been implemented
  public ShooterSuperstructure(Flywheel flywheel) {
    this.flywheel = flywheel;
  }
}
