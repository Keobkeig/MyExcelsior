package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeExtend extends CommandBase{
    private final Intake intake;
    
    public IntakeExtend(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.extend();
    }
}
