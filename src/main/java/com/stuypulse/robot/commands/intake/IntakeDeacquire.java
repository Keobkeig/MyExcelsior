package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeDeacquire extends CommandBase{
    private final Intake intake;

    public IntakeDeacquire(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deacquire();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
