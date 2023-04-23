package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ThenShoot extends CommandBase {
    private final Conveyor conveyor;
    private final Command alignmentCommand;

    private final ConveyorMode shootMode;

    public ThenShoot(Command alignmentCommand, Conveyor conveyor, ConveyorMode shootMode) {
        this.conveyor = conveyor;
        this.alignmentCommand = alignmentCommand;
        this.shootMode = shootMode;
        //takes from the command using ThenShoot
        m_requirements.addAll(this.alignmentCommand.getRequirements());
    }

    //default shootMode when no mode given
    public ThenShoot(Command alignmentCommand, Conveyor conveyor) {
        this(alignmentCommand, conveyor, ConveyorMode.SHOOT);
    }

    @Override
    public void initialize() {
        alignmentCommand.initialize();
    }
    
    @Override
    public void execute() {
        alignmentCommand.execute();
        if (alignmentCommand.isFinished()) {
            conveyor.setMode(shootMode);
        }
        else {
            conveyor.setMode(ConveyorMode.DEFAULT);
        }
    }

    @Override
    public boolean isFinished() {
        return alignmentCommand.isFinished() && conveyor.isEmpty(); 
    }

    @Override
    public void end(boolean interrupted) {
        alignmentCommand.end(interrupted);
        conveyor.setMode(ConveyorMode.DEFAULT);
    }
}
