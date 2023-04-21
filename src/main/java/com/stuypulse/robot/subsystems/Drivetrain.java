package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings.Drivetrain.Encoders;
import com.stuypulse.robot.constants.Settings.Drivetrain.Odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // public static enum Gear {
    //     HIGH(Value.kReverse),
    //     LOW(Value.kForward);

    //     private final Value value;

    //     private Gear(Value value) {
    //         this.value = value;
    //     }
    // }

    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    private final AHRS gyro;

    // private Gear gear;
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

        leftEncoder = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B);
        rightEncoder = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);
        setGrayhillDistancePerPulse(Encoders.GRAYHILL_DISTANCE_PER_PULSE);  //unit of distance per pulse conversion factor

        gyro = new AHRS(SPI.Port.kMXP); 

        //first parameter is optional and left out and is used to set the module number
        gearShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.Drivetrain.GEAR_SHIFT_FORWARD, Ports.Drivetrain.GEAR_SHIFT_REVERSE);

        odometry = new DifferentialDriveOdometry();
        field = new Field2d();
        reset(Odometry.STARTING_POSITION);
    }

    private void setGrayhillDistancePerPulse(double distance) {
        rightEncoder.setDistancePerPulse(distance);
        rightEncoder.reset();

        leftEncoder.setDistancePerPulse(distance);
        leftEncoder.reset();
    }
}
