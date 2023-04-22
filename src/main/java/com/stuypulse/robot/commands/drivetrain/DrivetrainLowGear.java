package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivetrainLowGear extends InstantCommand {

    private Drivetrain drivetrain;

    public DrivetrainLowGear(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setLowGear();
    }
}
