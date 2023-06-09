package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterExtendHood extends InstantCommand {
    private final Shooter shooter;

    public ShooterExtendHood(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.extendHood();
    }
}
