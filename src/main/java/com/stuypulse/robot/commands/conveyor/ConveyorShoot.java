package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorSetMode;
import com.stuypulse.robot.subsystems.Conveyor;

public class ConveyorShoot extends ConveyorSetMode {

    public ConveyorShoot(Conveyor conveyor) {
        super(conveyor, ConveyorMode.SHOOT);
    }

    @Override
    public boolean isFinished() {
        return conveyor.isEmpty();
    }
}
