package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.constants.Settings.Drivetrain.Stalling;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;

import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDrive extends CommandBase{
    private final Drivetrain drivetrain;
    private final Gamepad gamepad;
    
    private final BStream stalling;
    private final IStream speed;
    private final IStream angle;

    public DrivetrainDrive(Drivetrain drivetrain, Gamepad gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;

        stalling = Stalling.STALL_DETECTION
                .and(drivetrain::isStalling)
                .filtered(new BDebounceRC.Both(Settings.Drivetrain.Stalling.DEBOUNCE_TIME)); 

        /*
         * speed and angle are taken from speed and angle methods
         * filter: deadband, spow to power, and a lowpass filter 
         */
        speed = IStream.create(() -> gamepad.getRightTrigger() - gamepad.getLeftTrigger())
                        .filtered(
                            x -> SLMath.deadband(x, Settings.Drivetrain.SPEED_DEADBAND.get()),
                            x -> SLMath.spow(x, Settings.Drivetrain.SPEED_POWER.get()),
                            new LowPassFilter(Settings.Drivetrain.SPEED_FILTER)
                        );

        angle = IStream.create(() -> gamepad.getLeftX())
                        .filtered(
                            x -> SLMath.deadband(x, Settings.Drivetrain.ANGLE_DEADBAND.get()),
                            x -> SLMath.spow(x, Settings.Drivetrain.ANGLE_POWER.get()),
                            new LowPassFilter(Settings.Drivetrain.ANGLE_FILTER)
                        );

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (gamepad.getRawLeftButton()) {
            drivetrain.setLowGear();
            drivetrain.arcadeDrive(speed.get() - 0.1, angle.get());
        } 
        else {
            drivetrain.setLowGear();
            drivetrain.arcadeDrive(speed.get(), angle.get());
        }
        //TODO - add curvatureDrive
    }

    @Override
    public boolean isFinished() {
        return false; //NEVER OFF RAH
    }
}
