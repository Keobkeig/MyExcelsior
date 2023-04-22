package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorSetMode;
import com.stuypulse.robot.subsystems.Conveyor;

public class ConveyorForceIntake extends ConveyorSetMode{

    public ConveyorForceIntake(Conveyor conveyor, ConveyorMode mode) {
        super(conveyor, ConveyorMode.EJECT);
    }
    
}
