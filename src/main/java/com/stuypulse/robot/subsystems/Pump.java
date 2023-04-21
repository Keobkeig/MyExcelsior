package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pump extends SubsystemBase {
    private final SmartBoolean enabled;
    private final Compressor compressor;

    public Pump() {
        enabled = new SmartBoolean("Pump/Compressor Enabled", true);
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        
        stop();
    }

    public boolean getCompressing() {
        return compressor.isEnabled();
    }
    public void compress() {
        this.set(true);
    }

    public void stop() {
        this.set(false);
    }

    public void set(boolean compressing) {
        enabled.set(compressing);
    }

    @Override
    public void periodic() {
        if (enabled.get()) {
            compressor.enableDigital();
        }
        else {
            compressor.disable();
        }

    }
}
