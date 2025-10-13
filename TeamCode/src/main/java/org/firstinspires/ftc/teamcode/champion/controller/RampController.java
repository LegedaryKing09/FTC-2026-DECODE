package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RampController {

    public static String RANGE_SERVO_NAME = "ramp";
    public static double RANGE_SERVO_MIN_POSITION = 0.0;
    public static double RANGE_SERVO_MAX_POSITION = 1.0;
    public static double RANGE_SERVO_CENTER_POSITION = 0.5;

    // 240-degree specific constants
    public static double MAX_ANGLE_DEGREES = 240.0;
    public static double MIN_ANGLE_DEGREES = 0.0;
    public static double CENTER_ANGLE_DEGREES = 120.0;

    // Common angle positions
    public static double ANGLE_0_DEG = 0.0;
    public static double ANGLE_60_DEG = 60.0;
    public static double ANGLE_120_DEG = 120.0;
    public static double ANGLE_180_DEG = 180.0;
    public static double ANGLE_240_DEG = 240.0;

    private final Servo ramp;


    public RampController(LinearOpMode opMode) {
        ramp = opMode.hardwareMap.get(Servo.class, RANGE_SERVO_NAME);
        // Uncomment the line below if you need to reverse the servo direction
        ramp.setDirection(Servo.Direction.REVERSE);
    }

    // Position-based methods (0.0 to 1.0)
    public void setPosition(double position) {
        // Clamp position to valid range
        position = Math.max(RANGE_SERVO_MIN_POSITION, Math.min(RANGE_SERVO_MAX_POSITION, position));
        ramp.setPosition(position);
    }

    public double getPosition() {
        return ramp.getPosition();
    }

    // Angle-based methods (0 to 240 degrees)
    public void setAngle(double angleDegrees) {
        // Clamp angle to valid range
        angleDegrees = Math.max(MIN_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, angleDegrees));
        // Convert angle to position (0-240° maps to 0.0-1.0)
        double position = angleDegrees / MAX_ANGLE_DEGREES;
        setPosition(position);
    }

    public double getAngle() {
        // Convert position to angle
        return getPosition() * MAX_ANGLE_DEGREES;
    }

    // Preset position methods
    public void setToMin() {
        setPosition(RANGE_SERVO_MIN_POSITION);
    }

    public void setToMax() {
        setPosition(RANGE_SERVO_MAX_POSITION);
    }

    public void setToCenter() {
        setPosition(RANGE_SERVO_CENTER_POSITION);
    }

    public void setToQuarter() {
        setPosition(0.25);
    }

    public void setToThreeQuarter() {
        setPosition(0.75);
    }

    // Preset angle methods
    public void setTo0Degrees() {
        setAngle(ANGLE_0_DEG);
    }

    public void setTo60Degrees() {
        setAngle(ANGLE_60_DEG);
    }

    public void setTo120Degrees() {
        setAngle(ANGLE_120_DEG);
    }

    public void setTo180Degrees() {
        setAngle(ANGLE_180_DEG);
    }

    public void setTo240Degrees() {
        setAngle(ANGLE_240_DEG);
    }

    // Incremental movement methods
    public void incrementAngle(double degrees) {
        double currentAngle = getAngle();
        setAngle(currentAngle + degrees);
    }

    public void decrementAngle(double degrees) {
        incrementAngle(-degrees);
    }

    // Utility methods
    public boolean isAtMinPosition() {
        return Math.abs(getPosition() - RANGE_SERVO_MIN_POSITION) < 0.01;
    }

    public boolean isAtMaxPosition() {
        return Math.abs(getPosition() - RANGE_SERVO_MAX_POSITION) < 0.01;
    }

    public boolean isAtCenter() {
        return Math.abs(getPosition() - RANGE_SERVO_CENTER_POSITION) < 0.01;
    }

    public boolean isAtAngle(double targetAngle, double toleranceDegrees) {
        return Math.abs(getAngle() - targetAngle) <= toleranceDegrees;
    }
}