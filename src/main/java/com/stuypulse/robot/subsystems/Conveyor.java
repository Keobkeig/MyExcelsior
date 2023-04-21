package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

    public enum Direction {
        FORWARD,
        FORWARD_SLOW,
        STOPPED,
        REVERSE,
    }

    private ConveyorMode mode;

    private final CANSparkMax topBeltMotor;
    private final CANSparkMax bottomBeltMotor;
    
    private final ColorSensor colorSensor;
    private final DigitalInput topIRsensor;

    private final BStream empty;
    private final BStream newBall;

    private Direction topBeltDirection;
    private Direction gandalfDirection;

    public Conveyor(ColorSensor colorSensor) {
        topBeltMotor = new CANSparkMax(Ports.Conveyor.TOP_BELT_MOTOR, MotorType.kBrushless);
        Motors.Conveyor.TOP_BELT.configure(topBeltMotor); //configure CANSparkMaxs for unique motors
        bottomBeltMotor = new CANSparkMax(Ports.Conveyor.GANDALF_MOTOR,MotorType.kBrushless);
        Motors.Conveyor.GANDALF.configure(topBeltMotor);

        this.colorSensor = colorSensor;
        topIRsensor = new DigitalInput(Ports.Conveyor.TOP_BELT_IR_SENSOR);

        empty = BStream.create(this::hasTopBeltBall)
                        .or(this::hasAnyBall)
                        .not()
                        .polling(0.01);

        newBall = BStream.create(this::hasTopBeltBall)
                          .polling(0.01);
    
        setTopBeltDirection(Direction.STOPPED);
        setGandalfDirection(Direction.STOPPED);
        setMode(ConveyorMode.DEFAULT);
    }

    //modes of conveyor
    public void setMode(ConveyorMode mode) {
        this.mode = mode;
    }

    //getting/setting directions of motors
    public void setTopBeltDirection(Direction topBeltDirection) {
        this.topBeltDirection = topBeltDirection;
        switch (topBeltDirection) {
            case FORWARD:
                topBeltMotor.set(+Settings.Conveyor.TOP_BELT_SPEED.get());
                break;
            case FORWARD_SLOW:
                topBeltMotor.set(
                        Settings.Conveyor.TOP_BELT_SPEED.get() * Settings.Conveyor.SLOW_MUL.get());
                break;
            case STOPPED:
                topBeltMotor.stopMotor();
                break;
            case REVERSE:
                topBeltMotor.set(-Settings.Conveyor.TOP_BELT_SPEED.get());
                break;
        }
    }

    public void setGandalfDirection(Direction gandalfDirection) {
        this.gandalfDirection = gandalfDirection;
        switch (gandalfDirection) {
            case FORWARD: 
                bottomBeltMotor.set(+Settings.Conveyor.ACCEPT_SPEED.get());
                break;
            case FORWARD_SLOW:
                bottomBeltMotor.set(Settings.Conveyor.ACCEPT_SPEED.get() * Settings.Conveyor.SLOW_MUL.get());
                break;
            case STOPPED:
                bottomBeltMotor.stopMotor();
                break;
            case REVERSE:
                bottomBeltMotor.set(Settings.Conveyor.REJECT_SPEED.get());
                break;
        }
    }

    public Direction getTopBeltDirection() {
        return topBeltDirection;
    }

    public Direction getGandalfDirection() {
        return gandalfDirection;
    }


    //Sensor logic 
    public boolean hasTopBeltBall() {
        return !topIRsensor.get();
    }
    
    public boolean hasAnyBall() {
        return colorSensor.hasBall() || topIRsensor.get();
    }

    public boolean hasAllianceBall() {
        return colorSensor.hasAllianceBall();
    }

    public boolean hasOpponentBall() {
        return colorSensor.hasOpponentBall();
    }

    public boolean hasNewBall() {
        return newBall.get();
    }

    public boolean isEmpty() {
        return empty.get();
    }

    public boolean isFull() {
        return hasTopBeltBall() && hasAllianceBall();
    }

    @Override
    public void periodic() {
        mode.run(this);

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Debug/Conveyor/Top Belt", topBeltMotor.get());
            SmartDashboard.putNumber("Debug/Conveyor/Gandalf Motor", bottomBeltMotor.get());
            SmartDashboard.putBoolean("Debug/Conveyor/Top IR", hasTopBeltBall());
        }
    }
}
