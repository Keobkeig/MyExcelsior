package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDSet extends InstantCommand{
    private LEDColor color;
    private LEDController controller;

    public LEDSet(LEDController controller, LEDColor color) {
        this.controller = controller;
        this.color = color;
    }

    @Override
    public void initialize() {
        controller.setColor(color);
    }
}
