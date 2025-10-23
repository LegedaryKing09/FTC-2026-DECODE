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

    // 170-degree specific constants
    public static double MAX_ANGLE_DEGREES = 170.0;
    public static double MIN_ANGLE_DEGREES = 0.0;
    public static double CENTER_ANGLE_DEGREES = 85.0;

    // Common angle positions
    public static double ANGLE_0_DEG = 0.0;
    public static double ANGLE_42_5_DEG = 42.5;
    public static double ANGLE_85_DEG = 85.0;
    public static double ANGLE_127_5_DEG = 127.5;
    public static double ANGLE_170_DEG = 170.0;

    private final Servo ramp;


    public RampController(LinearOpMode opMode) {
        ramp = opMode.hardwareMap.get(Servo.class, RANGE_SERVO_NAME);
        // Uncomment the line below if you need to reverse the servo direction
        ramp.setDirection(Servo.Direction.FORWARD);
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

    // Angle-based methods (0 to 170 degrees)
    public void setAngle(double angleDegrees) {
        // Clamp angle to valid range
        angleDegrees = Math.max(MIN_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, angleDegrees));
        // Convert angle to position (0-170° maps to 0.0-1.0)
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

    public void setTo42_5Degrees() {
        setAngle(ANGLE_42_5_DEG);
    }

    public void setTo85Degrees() {
        setAngle(ANGLE_85_DEG);
    }

    public void setTo127_5Degrees() {
        setAngle(ANGLE_127_5_DEG);
    }

    public void setTo170Degrees() {
        setAngle(ANGLE_170_DEG);
    }

    // Incremental movement methods (still 2.5 degrees per increment)
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