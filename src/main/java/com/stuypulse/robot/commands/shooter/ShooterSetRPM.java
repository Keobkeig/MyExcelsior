package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetRPM extends InstantCommand {
    private final Shooter shooter;
    private Number speed;

    public ShooterSetRPM(Shooter shooter, Number speed) {
        this.speed = speed;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterRPM(speed);
    }

}
