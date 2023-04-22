package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorSetMode;
import com.stuypulse.robot.subsystems.Conveyor;

public class ConveyorStop extends ConveyorSetMode {

    public ConveyorStop(Conveyor conveyor) {
        super(conveyor, ConveyorMode.STOPPED);
    }
}
