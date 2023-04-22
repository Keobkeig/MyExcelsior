package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorSetMode;
import com.stuypulse.robot.subsystems.Conveyor;

public class ConveyorForceEject extends ConveyorSetMode {

    public ConveyorForceEject(Conveyor conveyor) {
        super(conveyor, ConveyorMode.EJECT);
    }
}
