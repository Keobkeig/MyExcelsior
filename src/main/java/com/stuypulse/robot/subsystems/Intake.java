package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Conveyor.Direction;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax motor;
    private final DoubleSolenoid solenoid;
    
    private final Conveyor conveyor;
    private final IFilter speedFilter;
    private double speed;

    private boolean ignoreConveyor;
    
    public Intake(Conveyor conveyor) {
        motor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        Motors.INTAKE.configure(motor);
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM ,Ports.Intake.SOLENOID_FORWARD, Ports.Intake.SOLENOID_REVERSE);

        speedFilter = new LowPassFilter(Settings.Intake.SPEED_FILTERING);

        this.conveyor = conveyor;
        ignoreConveyor = false;
    }

    public void extend() {
        solenoid.set(Value.kForward);

    }
    public void retract() {
        solenoid.set(Value.kReverse);
    }

    public void setMotor(double speed) {
        this.speed = speed;
    }

    public void stop() {
        this.speed = 0.0;
    }

    public void acquire() {
        this.speed = +Settings.Intake.ACQUIRE_SPEED.get();
    }
    
    public void deacquire() {
        if (DriverStation.isAutonomous()) {
            this.speed = -Settings.Intake.ACQUIRE_SPEED.get();
        } 
        else {
            this.speed = Settings.Intake.DEACQUIRE_SPEED.get();
        }
    }
    
    //logic using conveyor
    public void setIgnoreConveyor(boolean ignore) {
        ignoreConveyor = ignore;
    }

    public boolean getShouldStop() {
        if (ignoreConveyor) {
            return false;
        }
        else {
            return conveyor.getGandalfDirection() == Direction.STOPPED && conveyor.hasAnyBall();
        }
    }

    public boolean getShouldSlow() {
        if (ignoreConveyor) {
            return false;
        }
        else {
            return conveyor.getGandalfDirection() != Direction.STOPPED && conveyor.hasAnyBall();
        }
    }

    public boolean getShouldRetract() {
        if (ignoreConveyor) {
            return false;
        }
        else {
            return Settings.Intake.AUTO_RETRACT.get() && !DriverStation.isAutonomous() && conveyor.isFull();
        }
    }

    @Override
    public void periodic() {
        //applies filter
        double motorSpeed = speedFilter.get(speed);

        // conditions 
        if (0.0 <= motorSpeed && getShouldStop()) {
            motor.stopMotor();
        } else if (0.0 <= motorSpeed && getShouldSlow()) {
            motor.set(motorSpeed * 0.75);
        } else {
        motor.set(motorSpeed);
        }

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Debug/Intake/Motor Speed", motor.get());
            SmartDashboard.putBoolean("Debug/Intake/Extended", solenoid.get() == Value.kForward);
        }
    }
}
