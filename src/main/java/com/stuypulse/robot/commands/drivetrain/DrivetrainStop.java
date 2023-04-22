package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainStop extends CommandBase {
    private final Drivetrain drivetrain;
    
    public DrivetrainStop(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
