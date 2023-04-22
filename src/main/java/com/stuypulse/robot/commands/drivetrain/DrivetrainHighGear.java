package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivetrainHighGear extends InstantCommand {
    private Drivetrain drivetrain;

    public DrivetrainHighGear(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setHighGear();
    }
}
