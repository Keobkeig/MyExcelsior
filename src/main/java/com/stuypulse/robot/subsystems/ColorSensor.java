package com.stuypulse.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.ColorSensor.BallRGB;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    /*
    *  * Detects what color ball is in the Conveyor.
*
* Contains:
*      - getCurrentBall()
*         - Gets current ball seen by subsystem.
*         - Returns CurrentBall enum
*      - isConnected()
*         - Checks if the Color Sensor is connected
*         - Returns false if not, true if yes.
*      - hasAllianceBall()
*         - Checks if there is a ball and the ball is the correct alliance color
*         - Returns false if no ball or wrong Alliance color
*      - hasOpponentBall()
*         - Checks if there is a ball present and the ball is an opponent ball
*         - Returns false if no ball or is alliance ball
*      - hasBall()
*         - Checks if there is a ball present
*
     */

    private static class Sensor {
        private final ColorSensorV3 colorSensor;
        private boolean connected;
        private Color color;

        public Sensor() {
            colorSensor = new ColorSensorV3(Ports.ColorSensor.COLOR_SENSOR);
            connected = false;
            color = Color.kBlack;
        }

        public void update() {
            this.connected = Settings.ColorSensor.ENABLED.get(); 

            //multiple checks to see if still connected
            if(this.connected) {
                this.connected &= Settings.ColorSensor.AUTO.get() || !DriverStation.isAutonomous();
            }
            if(this.connected) {
                this.connected &= colorSensor.isConnected();
            }

            //color updating if connected
            if(this.connected) {
                this.color = colorSensor.getColor();
            }
            else
                this.color = Color.kBlack;
        }
    }

    public enum BallColor {
        RED_BALL,
        BLUE_BALL,
    }

    private BallColor targetBallColor;
    private final Sensor sensor;
    private final DigitalInput ballIRsensor;

    private final BStream alliance;
    private final BStream opponent;
    
    public ColorSensor() {
        sensor = new Sensor();
        ballIRsensor = new DigitalInput(Ports.ColorSensor.BALL_IR_SENSOR);

        alliance = BStream.create(() -> hasBall())
                            .and(() -> getCurrentBallColor() == getTargetBallColor())
                            .filtered(new BDebounce.Rising(Settings.ColorSensor.DEBOUNCE_TIME))
                            .polling(0.01);

        opponent = BStream.create(() -> hasBall())
                            .and(() -> getCurrentBallColor() != getTargetBallColor())
                            .filtered(new BDebounce.Falling(Settings.ColorSensor.DEBOUNCE_TIME))
                            .polling(0.01);

        getTargetBallUpdate();
    }

    public boolean hasBall() {
        return !ballIRsensor.get();
    }

    public BallColor getTargetBallUpdate() {
        switch (DriverStation.getAlliance()) {
            default:
                Settings.reportWarning("Driverstation.getAlliance() returned invalid colour!");
            case Red:
                return targetBallColor = BallColor.RED_BALL;
            case Blue:
                return targetBallColor = BallColor.BLUE_BALL;
        }
    }

    public BallColor getTargetBallColor() {
        return targetBallColor;
    }

    //color check via sensor 
    private static double getColorDistance(Color a, Color b) {
        double distanceR = a.red - b.red;
        double distanceG = a.green - b.green;
        double distanceB = a.blue - b.blue;
        return Math.pow(distanceR, 2) +  Math.pow(distanceB, 2) +  Math.pow(distanceG, 2);
    }

    private Color getRawColor() {
        return sensor.color;
    }

    public BallColor getCurrentBallColor() {
        double redError = getColorDistance(getRawColor(), BallRGB.RED);
        double blueError = getColorDistance(getRawColor(), BallRGB. BLUE);

        //create bias for given alliance colour
        switch (getTargetBallColor()) {
            case RED_BALL:
                redError /= Settings.ColorSensor.TARGET_BIAS.get();
                break;
            case BLUE_BALL:
                blueError /= Settings.ColorSensor.TARGET_BIAS.get();
                break;
        }

        //lower error = current colour 
        return redError < blueError ? BallColor.RED_BALL : BallColor.BLUE_BALL;
    }

    //COLOURSENSOR METHODS FOR OTTHER SUBSYSTEMS TO USE IN LOGIC 
    private boolean isConnected() {
        return sensor.connected;
    }

    public boolean hasAllianceBall() {
        if (!isConnected()) {
            return false;
        }
        else {
            return alliance.get();
        }
    }

    public boolean hasOpponentBall() {
        if (!isConnected()) {
            return false;
        }
        else {
            return opponent.get();
        }
    }

    public boolean hasBall(BallColor target) {
        if (target == getTargetBallColor()) {
            return hasAllianceBall();
        } else {
            return hasOpponentBall();
        }
    }

    @Override
    public void periodic() {
        sensor.update();

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putBoolean("Debug/Color Sensor/Is Connected", isConnected());

            SmartDashboard.putNumber("Debug/Color Sensor/Color R", getRawColor().red);
            SmartDashboard.putNumber("Debug/Color Sensor/Color G", getRawColor().green);
            SmartDashboard.putNumber("Debug/Color Sensor/Color B", getRawColor().blue);

            SmartDashboard.putBoolean("Debug/Color Sensor/Has Any Ball", hasBall());
            SmartDashboard.putBoolean("Debug/Color Sensor/Has Alliance Ball", hasAllianceBall());
            SmartDashboard.putBoolean("Debug/Color Sensor/Has Opponent Ball", hasOpponentBall());
        }
    }
}
