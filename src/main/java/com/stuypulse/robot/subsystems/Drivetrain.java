package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors.Config;
import com.stuypulse.robot.constants.Settings.Drivetrain.Encoders;
import com.stuypulse.robot.constants.Settings.Drivetrain.Odometry;
import com.stuypulse.robot.constants.Settings.Drivetrain.Stalling;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    public static enum Gear {
        HIGH(Value.kReverse),
        LOW(Value.kForward);

        private final Value value;

        private Gear(Value value) {
            this.value = value;
        }
    }

    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    private final AHRS gyro;

    private Gear gear;
    private final DoubleSolenoid gearShift;
    private final DifferentialDrive drivetrain;

    private final DifferentialDriveOdometry odometry;
    private final Field2d field;

    public Drivetrain() {
        leftMotors = new CANSparkMax[] {
            new CANSparkMax (Ports.Drivetrain.LEFT_TOP, MotorType.kBrushless),
            new CANSparkMax (Ports.Drivetrain.LEFT_MIDDLE, MotorType.kBrushless),
            new CANSparkMax (Ports.Drivetrain.LEFT_BOTTOM, MotorType.kBrushless),
        };

        rightMotors = new CANSparkMax[] {
            new CANSparkMax (Ports.Drivetrain.RIGHT_TOP, MotorType.kBrushless),
            new CANSparkMax (Ports.Drivetrain.RIGHT_MIDDLE, MotorType.kBrushless),
            new CANSparkMax (Ports.Drivetrain.RIGHT_BOTTOM, MotorType.kBrushless),
        };

        setMotorConfig(Motors.Drivetrain.LEFT, Motors.Drivetrain.RIGHT);
        setHighGear();
        
        leftEncoder = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B);
        rightEncoder = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);
        setGrayhillDistancePerPulse(Encoders.GRAYHILL_DISTANCE_PER_PULSE);  //unit of distance per pulse conversion factor

        gyro = new AHRS(SPI.Port.kMXP); 

        //first parameter is optional and left out and is used to set the module number
        gearShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.Drivetrain.GEAR_SHIFT_FORWARD, Ports.Drivetrain.GEAR_SHIFT_REVERSE);

        drivetrain = new DifferentialDrive(new MotorControllerGroup(leftMotors), new MotorControllerGroup(rightMotors));
        odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDistance(), getRightDistance());
        field = new Field2d();
        reset(Odometry.STARTING_POSITION);
    }

    //gear
    public Gear getGear() {
        return gear;
    }

    public void setGear(Gear gear) {
        gearShift.set(gear.value);
        this.gear = gear;
    }

    public void setLowGear() {
        setGear(Gear.LOW);
    }

    public void setHighGear() {
        setGear(Gear.HIGH);
    }

    //config
    private void setMotorConfig(Config left, Config right) {
        //left is inverted, right is not
        leftEncoder.setReverseDirection(Settings.Drivetrain.Encoders.GRAYHILL_INVERTED ^ left.INVERTED);
        rightEncoder.setReverseDirection(Settings.Drivetrain.Encoders.GRAYHILL_INVERTED ^ right.INVERTED);
        for (CANSparkMax motor: leftMotors) {
            left.configure(motor);
        }
        for (CANSparkMax motor: rightMotors) {
            right.configure(motor);
        }
    }

    private void setGrayhillDistancePerPulse(double distance) {
        rightEncoder.setDistancePerPulse(distance);
        rightEncoder.reset();

        leftEncoder.setDistancePerPulse(distance);
        leftEncoder.reset();
    }

    //get distances of encoders 
      public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    //get velocities of encoders
    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    public double getRightVelocity() {
        return rightEncoder.getRate();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    //return angles from gyro
    public double getRawGyroAngle() {
        return gyro.getAngle();
    }

    public Angle getGyroAngle() {
        return Angle.fromDegrees(getRawGyroAngle());
    }
    
    //return angles from encoders (why?)
    private double getRawEncoderAngle() {
        //double distance = getLeftDistance() - getRightDistance(); (version that is doing neg(since inverted) - pos)
        double distance = getRightDistance() - getLeftDistance() ; //pos - neg so no need to do .neg later
        return Math.toDegrees(distance / Settings.Drivetrain.TRACK_WIDTH);
        
    }
    public Angle getEncoderAngle() {
        return Angle.fromDegrees(getRawEncoderAngle());
    }

    //depends on whehter we want to use gyro or encoders for angles
    public Angle getAngle() {
        return Settings.Drivetrain.USING_GYRO ? getGyroAngle() : getEncoderAngle();
    }

    //odometry 
    private void updateOdometry() {
        odometry.update(getRotation2d(), getRightDistance(), getLeftDistance()); 
    }

    private Rotation2d getRotation2d() {
        return getAngle().getRotation2d();
    } 

    /*
     * yaw = x
     * pitch = y
     * roll = z 
     */
    public Angle getRoll() {
        return Angle.fromDegrees(gyro.getRoll());
    }

    public Angle getPitch() {
        return Angle.fromDegrees(gyro.getPitch());
    }

    public Angle getYaw() {
        return Angle.fromDegrees(gyro.getYaw());
    }

    public Field2d getField2d() {
        return field;
    }

    public Pose2d getPose() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    public void reset(Pose2d location) {
        gyro.reset();
        leftEncoder.reset();
        rightEncoder.reset();
        odometry.resetPosition(getRotation2d(), 0, 0, location);
    }

    public void reset() {
        reset(getPose());
    }

    //voltage getters and drive 
    public double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public double getLeftVoltage() {
        return leftMotors[0].get() * getBatteryVoltage();
    }

    public double getRightVoltage() {
        return rightMotors[0].get() * getBatteryVoltage();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        for (MotorController motor : leftMotors) {
            motor.setVoltage(leftVolts);
        }
        for (MotorController motor : rightMotors) {
            motor.setVoltage(rightVolts);
        }
        drivetrain.feed();
    }

    public double getLeftCurrentAmps() {
        double amps = 0.0;
        for (CANSparkMax motor: leftMotors) {
            amps += Math.abs(motor.getOutputCurrent());
        }
        return amps / leftMotors.length;
    }

    public double getRightCurrentAmps() {
        double amps = 0.0;
        for (CANSparkMax motor: rightMotors) {
            amps += Math.abs(motor.getOutputCurrent());
        }
        return amps / rightMotors.length;
    }

    public double getCurrentAmps() {
        return (getLeftCurrentAmps() + getRightCurrentAmps()) / 2.0;
    }

    public boolean isLeftStalling() {
        boolean highGear = getGear() == Gear.HIGH;
        boolean current = getLeftCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(leftMotors[0].get()) > Stalling.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(getLeftVelocity()) < Stalling.SCIBORGS_THRESHOLD;
        return highGear && (current || output) && velocity;
        
    }

    public boolean isRightStalling() {
        boolean highGear = getGear() == Gear.HIGH;
        boolean current = getRightCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(rightMotors[0].get()) > Stalling.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(getRightVelocity()) < Stalling.SCIBORGS_THRESHOLD;
        return highGear && (current || output) && velocity;
    }

    //conditions for stalling: 
    //is in highGear? and greater then stalling threshold for current/amps? and less than velocity threshold?
    public boolean isStalling() {
        return isLeftStalling() || isRightStalling();
    }

    public void stop() {
        drivetrain.stopMotor();
    }

    public void tankDrive(double left, double right) {
        drivetrain.tankDrive(left, right, false);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation, false);
    }

    //TO-DO: Add curvatureDrive()

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose());

        // Smart Dashboard Information
        if (Settings.DEBUG_MODE.get()) {

            SmartDashboard.putNumber("Debug/Drivetrain/Roll (deg)", getRoll().toDegrees());

            SmartDashboard.putData("Debug/Drivetrain/Field", field);
            SmartDashboard.putBoolean("Debug/Drivetrain/High Gear", getGear().equals(Gear.HIGH));
            SmartDashboard.putNumber("Debug/Drivetrain/Odometer X Position (m)", getPose().getX());
            SmartDashboard.putNumber("Debug/Drivetrain/Odometer Y Position (m)", getPose().getY());
            SmartDashboard.putNumber(
                    "Debug/Drivetrain/Odometer Rotation (deg)",
                    getPose().getRotation().getDegrees());

            SmartDashboard.putNumber("Debug/Drivetrain/Motor Voltage Left (V)", getLeftVoltage());
            SmartDashboard.putNumber("Debug/Drivetrain/Motor Voltage Right (V)", getRightVoltage());

            SmartDashboard.putNumber("Debug/Drivetrain/Distance Traveled (m)", getDistance());
            SmartDashboard.putNumber(
                    "Debug/Drivetrain/Distance Traveled Left (m)", getLeftDistance());
            SmartDashboard.putNumber(
                    "Debug/Drivetrain/Distance Traveled Right (m)", getRightDistance());

            SmartDashboard.putNumber("Debug/Drivetrain/Velocity (m per s)", getVelocity());
            SmartDashboard.putNumber("Debug/Drivetrain/Velocity Left (m per s)", getLeftVelocity());
            SmartDashboard.putNumber(
                    "Debug/Drivetrain/Velocity Right (m per s)", getRightVelocity());

            SmartDashboard.putNumber("Debug/Drivetrain/Current Left (amps)", getLeftCurrentAmps());
            SmartDashboard.putNumber(
                    "Debug/Drivetrain/Current Right (amps)", getRightCurrentAmps());

            SmartDashboard.putNumber("Debug/Drivetrain/Angle NavX (deg)", getAngle().toDegrees());
            SmartDashboard.putNumber(
                    "Debug/Drivetrain/Encoder Angle (deg)", getEncoderAngle().toDegrees());
        }
    }
}
