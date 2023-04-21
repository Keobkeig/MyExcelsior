package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Climber.Stalling;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public enum Tilt {
        MAX_TILT(Value.kForward),
        NO_TILT(Value.kReverse);

        private final Value extended;

        private Tilt(Value extended) {
            this.extended = extended;
        }
    }

    private final CANSparkMax climber;
    private final Debouncer stalling; 

    private final DoubleSolenoid tilter;

    private final DigitalInput left;
    private final DigitalInput right;

    public Climber() {
        climber = new CANSparkMax(Ports.Climber.MOTOR, MotorType.kBrushless);
        Motors.CLIMBER.configure(climber);
        stalling = new Debouncer(Stalling.DEBOUNCE_TIME, DebounceType.kBoth);
        tilter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.Climber.TILTER_FORWARD, Ports.Climber.TILTER_REVERSE);
        left = new DigitalInput(Ports.Climber.LEFT_LIMIT);
        right = new DigitalInput(Ports.Climber.RIGHT_LIMIT);
    }

    //motor controls
    public void forceLowerClimb() {
        climber.set(Settings.Climber.SLOW_SPEED.get());
    }

    public void setMotor(double speed) {
        //catching potential failures when setting motors
        if (speed != 0.0 && isStalling()) {
            DriverStation.reportError("[CRITICAL] Climber is stalling when attempting to move!", false);
            stalling.calculate(true);
            setMotorStop();
        }
        else if (speed < 0.0 && getHookClear()) {
            Settings.reportWarning("Climber attemted to run past bottom limit!");
            setMotorStop();
        }
        else {
            climber.set(speed);
        }
    }

    public void setMotorStop() {
        climber.stopMotor();
    }

    // tilt control
    public void setTilt(Tilt tilt) {
        tilter.set(tilt.extended);
    }

    // clearance checks
    public boolean getLeftClear() {
        return !left.get();
    }

    public boolean getRightClear() {
        return !right.get();
    }

    private boolean getHookClear() {
        return getLeftClear() && getRightClear();
    }


    // stall check
    private double getDutyCycle() {
        return climber.get();
    }

    private double getCurrentAmps() {
        return Math.abs(climber.getOutputCurrent());
    }

    private boolean isStalling() {
        boolean current = getCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(getDutyCycle()) > Stalling.DUTY_CYCLE_THRESHOLD;
        return Stalling.ENABLED.get() && stalling.calculate(output && current);
    }    

    @Override
    public void periodic() {
        if (isStalling()) {
            DriverStation.reportError(
                    "[CRITICAL] Climber is stalling when attempting to move!", false);
            setMotorStop();
        }
        // This method will be called once per scheduler run not perma
        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putBoolean("Debug/Climber/Stalling", isStalling());
            SmartDashboard.putNumber("Debug/Climber/Current Amps", getCurrentAmps());
            SmartDashboard.putBoolean(
                    "Debug/Climber/Max Tilt", tilter.get().equals(Value.kReverse));
            SmartDashboard.putNumber("Debug/Climber/Climber Speed", climber.get());
        }
    }
}
