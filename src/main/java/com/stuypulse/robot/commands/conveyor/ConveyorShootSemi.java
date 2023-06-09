package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorSetMode;
import com.stuypulse.robot.subsystems.Conveyor;

public class ConveyorShootSemi extends ConveyorSetMode {

    public ConveyorShootSemi(Conveyor conveyor) {
        super(conveyor, ConveyorMode.SEMI_AUTO);
    }

    public boolean isFinished() {
        return conveyor.isEmpty();
    }
}
