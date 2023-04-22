package com.stuypulse.robot.commands.conveyor.modes;


import com.stuypulse.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorSetMode extends CommandBase {
    protected final ConveyorMode mode;
    protected final Conveyor conveyor;
    
    public ConveyorSetMode(Conveyor conveyor, ConveyorMode mode) {
        this.conveyor = conveyor;
        this.mode = mode;

        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setMode(mode);
    }

    @Override
    public final void end(boolean interrupted) {
        conveyor.setMode(ConveyorMode.DEFAULT);
    }
}
