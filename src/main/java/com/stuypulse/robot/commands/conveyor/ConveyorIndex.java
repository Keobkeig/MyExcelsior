package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorSetMode;
import com.stuypulse.robot.subsystems.Conveyor;

public class ConveyorIndex extends ConveyorSetMode {

    public ConveyorIndex(Conveyor conveyor) {
        super(conveyor, ConveyorMode.INDEX);
    }
}
