package com.stuypulse.robot.subsystems;


import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.ColorSensor.BallColor;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.robot.util.TeleopButton;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final PWMSparkMax controller;

    private final StopWatch stopWatch;
    private double manualTime;

    private final RobotContainer container;
    private LEDColor manualColor;

    public LEDController(RobotContainer container) {
        controller = new PWMSparkMax(Ports.LEDController.PWM_PORT);
        stopWatch = new StopWatch();
        this.container = container;
        setLEDConditions();
        setColor(LEDColor.OFF);
    }

    public void setColor(LEDColor color, double time) {
        setColor(color, time);
        stopWatch.reset();
    }

    public void setColor(LEDColor color) {
        setColor(color, Settings.LED.MANUAL_UPDATE_TIME);
    }

    private void setLEDConditions() {
        new TeleopButton(() -> container.colorSensor.hasBall(BallColor.RED_BALL))
                .whileTrue(new LEDSet(this, LEDColor.RED));
        new TeleopButton(() -> container.colorSensor.hasBall(BallColor.BLUE_BALL))
                .whileTrue(new LEDSet(this, LEDColor.BLUE));
    }

    public LEDColor getDefaultColor() {
        if (DriverStation.isTest() && container.pump.getCompressing()) return LEDColor.HEARTBEAT;

        // limit switches
        boolean left = container.climber.getLeftClear();
        boolean right = container.climber.getRightClear();
        if (left && right) {
            return LEDColor.PURPLE;
        } else if (left || right) {
            return LEDColor.RED;
        }

        // time based LEDs
        double time = DriverStation.getMatchTime(); // time remaining in a game
        if (time > Settings.LED.MIN_MATCH_TIME) {
            if (time < Settings.LED.END_GAME_TIME) return LEDColor.RED;
            if (time < Settings.LED.CLIMB_TIME) {
                double roll = Math.abs(container.drivetrain.getRoll().toDegrees());
                if (roll < 3.0) return LEDColor.RAINBOW.pulse();
                if (roll < 10.0) return LEDColor.BLUE;
                if (roll < 20.0) return LEDColor.PURPLE;
                if (roll < 50.0) return LEDColor.RED;
            }
        }

        if (Settings.LED.SWAP_RAINBOW.get()) {
            if (!container.conveyor.isFull()) return LEDColor.RAINBOW;
        } else {
            if (container.conveyor.isFull()) return LEDColor.RAINBOW;
        }

        double shooterError =
                Math.abs(container.shooter.getRawTargetRPM() - container.shooter.getShooterRPM());

        if (container.shooter.getRawTargetRPM() <= Settings.LED.RPM_ERROR_STEP) return LEDColor.OFF;
        if (shooterError <= 1.0 * Settings.LED.RPM_ERROR_STEP) return LEDColor.GREEN;
        if (shooterError <= 2.0 * Settings.LED.RPM_ERROR_STEP) return LEDColor.LIME;
        if (shooterError <= 3.0 * Settings.LED.RPM_ERROR_STEP) return LEDColor.YELLOW;
        if (shooterError <= 4.0 * Settings.LED.RPM_ERROR_STEP) return LEDColor.ORANGE;
        if (shooterError <= 5.0 * Settings.LED.RPM_ERROR_STEP) return LEDColor.RED;
        else return LEDColor.RED.pulse();
    }

    @Override
    public void periodic() {
        // If we called .setColor() recently, use that value
        if (DriverStation.isAutonomous() || stopWatch.getTime() < manualTime) {
            controller.set(manualColor.get());
        }
        // Otherwise use the default color
        else {
            controller.set(getDefaultColor().get());
        }
    }
}
