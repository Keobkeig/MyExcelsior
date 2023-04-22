package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorSetMode;
import com.stuypulse.robot.subsystems.Conveyor;

public class ConveyorShootSlow extends ConveyorSetMode {

    public ConveyorShootSlow(Conveyor conveyor) {
        super(conveyor, ConveyorMode.SHOOT_SLOW);
    }

    @Override
    public boolean isFinished() {
        return conveyor.isEmpty();
    }
}
