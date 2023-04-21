package com.stuypulse.robot.subsystems;
/*
 * A utility class meant for controlling a flywheel system (shooter, feeder, etc.) by driving it to
 * a reference rotations per minute.
 *
 * <p>Stores a simple feedforward model of the shooter based on the voltage-balance equation and a
 * PID controller to correct for any error.
 */

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDFlyWheel extends SubsystemBase {
    private double targetRPM; 

    private final List<CANSparkMax> motors;
    private final List<RelativeEncoder> encoders;

    private final SimpleMotorFeedforward feedforward;
    private final Controller feedbackController;

    public PIDFlyWheel(CANSparkMax motor, SimpleMotorFeedforward feedforward, Controller feedbackController) {
        this.motors = new ArrayList<>();
        this.encoders = new ArrayList<>();
        addFollower(motor);

        this.feedforward = feedforward;
        this.feedbackController = feedbackController;

        this.targetRPM = 0.0;
    }

    public PIDFlyWheel addFollower(CANSparkMax follower) {
        this.motors.add(follower);
        this.encoders.add(follower.getEncoder());
        return this;
    }

    public void stop() {
        setVelocity(0.0);
    }

    public void setVelocity(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    public double getVelocity() {
        double velocity = 0.0;
        for (RelativeEncoder encoder: encoders) {
            velocity += encoder.getVelocity();
        }
        return velocity / encoders.size();
    }

    @Override
    public void periodic() {
        //prevents going too crazyyy
        if (targetRPM < 200.0) {
            for (CANSparkMax motor: motors) {
                motor.stopMotor();
            }
        } 
        //uses feedforward and PID feedback to adjust voltages
        else {
            double ff = feedforward.calculate(this.targetRPM);
            double fb = feedbackController.update(this.targetRPM, getVelocity());

            for (CANSparkMax motor : this.motors) {
                motor.setVoltage(SLMath.clamp(ff + fb, 0, 16));
            }
        }
    }
}
