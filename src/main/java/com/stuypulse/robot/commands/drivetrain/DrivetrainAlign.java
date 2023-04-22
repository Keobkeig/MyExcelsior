package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.Drivetrain.Gear;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.IFuser;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainAlign extends CommandBase {
    private final Drivetrain drivetrain;

    //want until true once
    private final BStream finished;

    private IFilter speedAdjustFilter;

    private final IFuser angleError;
    private final IFuser distanceError;

    protected final Controller angleController;
    protected final Controller distanceController;

    public DrivetrainAlign(Drivetrain drivetrain, Camera camera) {
        this.drivetrain = drivetrain;

        angleError = new IFuser(Alignment.FUSION_FILTER, 
                            () -> camera.getXAngle().add(Angle.fromDegrees(Limelight.RING_YAW.get())).toDegrees(),
                            () -> drivetrain.getRawGyroAngle());

        distanceError = new IFuser(Alignment.SPEED_ADJ_FILTER,
                            () -> Settings.Limelight.RING_DISTANCE.get() - camera.getDistance(),
                            () -> drivetrain.getDistance());

        angleController = Alignment.Angle.getController();
        distanceController = Alignment.Speed.getController();

        speedAdjustFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);

        finished = BStream.create(camera::hasTarget)
                            .and(() -> Math.abs(drivetrain.getVelocity()) < Limelight.MAX_VELOCITY.get())
                            .and(() -> angleController.isDone(Limelight.MAX_ANGLE_ERROR.get()))
                            .and(() -> distanceController.isDone(Limelight.MAX_DISTANCE_ERROR.get()))
                            .filtered(new BDebounceRC.Rising(Limelight.DEBOUNCE_TIME));

        addRequirements(drivetrain);
    }    

    @Override
    public void initialize() {
        drivetrain.setLowGear();
        speedAdjustFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);
        angleError.reset();
        distanceError.reset();
    }

    private double getSpeedAdjustment() {
        double error = angleError.get() / Limelight.MAX_ANGLE_FOR_MOVEMENT.get();
        return speedAdjustFilter.get(error);
    }

    private double getSpeed() {
        double speed = distanceController.update(distanceError.get(), 0);
        return speed * getSpeedAdjustment();
    }

    private double getTurn() {
        return angleController.update(angleError.get(), 0);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(getSpeed(), getTurn());
    }

    @Override
    public boolean isFinished() {
        return finished.get();
    }

    public Command thenShoot(Conveyor conveyor) {
        return new ThenShoot(this, conveyor, ConveyorMode.SHOOT);
    }
}
