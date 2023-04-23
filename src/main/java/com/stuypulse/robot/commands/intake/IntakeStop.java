package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeStop extends CommandBase{
    private final Intake intake;

    public IntakeStop(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
