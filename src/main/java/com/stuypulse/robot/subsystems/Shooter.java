package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.filters.RateLimit;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final SmartNumber targetRPM;
    private final IFilter targetFilter;

    //motors represented in flywheels 
    private final PIDFlyWheel shooter;
    private final PIDFlyWheel feeder;

    private final Solenoid hood;

    public Shooter() {
        targetRPM = new SmartNumber("Shooter/ Target RPM", 0.0);
        //was supposed to be TimedRateLimit 
        targetFilter = new RateLimit(Settings.Shooter.MAX_TARGET_RPM_CHANGE)
                                    .then(new LowPassFilter(Settings.Shooter.CHANGE_RC));

        CANSparkMax shooterMotor = new CANSparkMax(Ports.Shooter.LEFT, MotorType.kBrushless);
        CANSparkMax shooterFollowerMotor = new CANSparkMax(Ports.Shooter.RIGHT, MotorType.kBrushless);

        shooter = new PIDFlyWheel(shooterMotor, Settings.Shooter.ShooterFF.getController(), Settings.Shooter.ShooterPID.getController());
        shooter.addFollower(shooterFollowerMotor);

        CANSparkMax feederMotor = new CANSparkMax(Ports.Shooter.FEEDER, MotorType.kBrushless);

        feeder = new PIDFlyWheel(feederMotor, Settings.Shooter.FeederFF.getController(), Settings.Shooter.FeederPID.getController());

        hood = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Shooter.HOOD_SOLENOID);

        Motors.Shooter.LEFT.configure(shooterMotor);
        Motors.Shooter.RIGHT.configure(shooterFollowerMotor);
        Motors.Shooter.FEEDER.configure(feederMotor);
    }

    // set/ do methods
    public void setShooterRPM(double speed) {
        targetRPM.set(speed);
    }

    public void extendHood() {
        hood.set(true);
    }

    public void retractHood(){
        hood.set(false);
    }

    // get/encoder methods
    public double getShooterRPM() {
        return shooter.getVelocity();
    }

    public double getFeederRPM() {
        return feeder.getVelocity();
    }

    public boolean isFenderMode() {
        return hood.get();
    }

    public double getRawTargetRPM() {
        return targetRPM.get();
    }

    public double getTargetRPM() {
        return targetFilter.get(getRawTargetRPM());
    }

    public boolean isReady() {
        return Math.abs(getShooterRPM() - getRawTargetRPM()) < Settings.Shooter.MAX_RPM_ERROR;
    }

    @Override
    public void periodic() {
        double goal = getTargetRPM();

        if (goal < Settings.Shooter.MIN_RPM) {
            shooter.stop();
            feeder.stop();
        } else {
            shooter.setVelocity(goal);
            feeder.setVelocity(goal * Settings.Shooter.FEEDER_MULTIPLER.get());
        }

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Debug/Shooter/Shooter RPM", getShooterRPM());
            SmartDashboard.putNumber("Debug/Shooter/Feeder RPM", getFeederRPM());
        }
    }
}

