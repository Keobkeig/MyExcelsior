package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.constants.Settings.Drivetrain.Motion;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.util.TrajectoryLoader;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.PIDController;


//ramsete commands takes 
public class DrivetrainRamsete extends RamseteCommand {
    protected boolean resetPosition;
    protected Trajectory trajectory;
    protected Drivetrain drivetrain;

    public DrivetrainRamsete(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            drivetrain::getPose,
            new RamseteController(),
            Motion.MOTOR_FEED_FORWARD,
            Motion.KINEMATICS,
            drivetrain:: getWheelSpeeds,
            new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD),
            new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD),
            drivetrain::tankDriveVolts, 
            drivetrain
        );

        resetPosition = true;
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    //other versions using String and String...
    public DrivetrainRamsete(Drivetrain drivetrain, String path) {
        this(drivetrain, TrajectoryLoader.getTrajectory(path));
    }

    public DrivetrainRamsete(Drivetrain drivetrain, String... paths) {
        this(drivetrain, TrajectoryLoader.getTrajectory(paths));
    }

    public DrivetrainRamsete RobotRelative() {
        resetPosition = true;
        return this;
    }

    public DrivetrainRamsete fieldRelative() {
        resetPosition = false;
        return this;
    }
    @Override
    public void initialize() {
        super.initialize();
        drivetrain.setHighGear();
        if (resetPosition) {
            drivetrain.reset(trajectory.getInitialPose());
        }
    }
}
