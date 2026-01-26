package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

    private ShooterCommands() {}

    public static Command runShooter(Shooter shooter) {
        return Commands.runEnd(
            () -> shooter.setDutyCycle(0.8), 
            () -> shooter.setNeutral(), 
            shooter
        );
    }
    
}
