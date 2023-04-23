package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//a command that runs pre-existing commands in order 
public class ShooterRingShot extends SequentialCommandGroup {
    public ShooterRingShot(Shooter shooter) {
        addCommands(new ShooterRetractHood(shooter));
        addCommands(new ShooterSetRPM(shooter, Settings.Shooter.RING_RPM));
    }
}
