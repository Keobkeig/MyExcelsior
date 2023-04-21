package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;
import com.stuypulse.stuylib.network.limelight.Limelight.LEDMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    private final Shooter shooter;
    private final Limelight limelight;

    public Camera(Shooter shooter) {
        this.shooter = shooter;
        limelight = Limelight.getInstance();

        for (int port: Settings.Limelight.PORTS) {
            PortForwarder.add(port,"limelight.local", port);
        }

        CameraServer.startAutomaticCapture();
    }
    
    public boolean hasAnyTarget(){
        return limelight.getValidTarget();
    }
    
    public boolean hasTarget(){
        return hasAnyTarget() && shooter.isReady() && (Settings.Limelight.MIN_VALID_DISTANCE < getDistance()) && (getDistance() < Settings.Limelight.MAX_VALID_DISTANCE);
    }

    /*
     * yaw = x
     * pitch = y
     * roll = z (un-used)
     */
    public Angle getXAngle() {
        if (!hasAnyTarget()) {
            Settings.reportWarning("Unable to find target when using getXAngle()");
            return Angle.kZero;
        }
        else {
            return Angle.fromDegrees(limelight.getTargetXAngle() + Settings.Limelight.LIMELIGHT_YAW.get());
        }
    }

    public Angle getYAngle() {
        if (!hasAnyTarget()) {
            Settings.reportWarning("Unable to find target when using getYAngle()");
            return Angle.kZero;
        }
        else {
            return Angle.fromDegrees(limelight.getTargetYAngle() + Settings.Limelight.LIMELIGHT_PITCH.get());
        }
    }

    public double getDistance() {
        if (!hasAnyTarget()) {
            Settings.reportWarning("Unable To Find Target! [getDistance() was called]");
            return Settings.Limelight.RING_DISTANCE.get();
        }
        else {
            return Settings.Limelight.CENTER_TO_HUB + Settings.Limelight.LIMELIGHT_TO_INTAKE + Settings.Limelight.HEIGHT_DIFFERENCE/ getYAngle().tan(); //edge of goal to limelight
        }
    }

    @Override
    public void periodic() {
        if (Settings.DEBUG_MODE.get() && hasAnyTarget()) {
            SmartDashboard.putNumber("Camera/Distance", getDistance());
        }

        if (!limelight.isConnected()) {
            Settings.reportWarning("Limelight Disconnected!");
        }

        if (DriverStation.isDisabled()) {
            limelight.setLEDMode(LEDMode.PIPELINE);
        } 
        else if (shooter.isFenderMode()) {
            limelight.setLEDMode(LEDMode.FORCE_ON);
        } 
        else {
            limelight.setLEDMode(LEDMode.FORCE_ON);
        }
    }
}
