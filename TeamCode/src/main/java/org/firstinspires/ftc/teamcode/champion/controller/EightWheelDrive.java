package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Controller for eight-wheel drive system with sensitivity curves
 */
public class EightWheelDrive {

    private DcMotor motor1Left, motor2Left;
    private DcMotor motor1Right, motor2Right;

    private boolean isFastMode = false;

    // Tunable parameters
    public double joystickDeadzone = 0.05;
    public double driveSensitivityCurve = 2.0;
    public double turnSensitivityCurve = 2.0;

    public double fastSpeedMultiplier = 0.8;
    public double fastTurnMultiplier = 0.6;
    public double slowSpeedMultiplier = 0.8;
    public double slowTurnMultiplier = 0.6;

    public EightWheelDrive(DcMotor l1, DcMotor l2, DcMotor r1, DcMotor r2) {
        motor1Left = l1;
        motor2Left = l2;
        motor1Right = r1;
        motor2Right = r2;

        // Initialize motors if they exist
        if (motor1Left != null) {
            motor1Left.setDirection(DcMotorSimple.Direction.FORWARD);
            motor1Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (motor2Left != null) {
            motor2Left.setDirection(DcMotorSimple.Direction.FORWARD);
            motor2Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (motor1Right != null) {
            motor1Right.setDirection(DcMotorSimple.Direction.REVERSE);
            motor1Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (motor2Right != null) {
            motor2Right.setDirection(DcMotorSimple.Direction.REVERSE);
            motor2Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Toggle between fast and slow speed modes
     */
    public void toggleSpeedMode() {
        isFastMode = !isFastMode;
    }

    /**
     * Set speed mode directly
     */
    public void setFastMode(boolean fast) {
        isFastMode = fast;
    }

    /**
     * Update drive motors based on joystick inputs
     */
    public void drive(double rawDrive, double rawTurn) {
        // Apply deadzone
        rawDrive = applyDeadzone(rawDrive, joystickDeadzone);
        rawTurn = applyDeadzone(rawTurn, joystickDeadzone);

        // Apply sensitivity curve
        double curveDrive = applySensitivityCurve(rawDrive, driveSensitivityCurve);
        double curveTurn = applySensitivityCurve(rawTurn, turnSensitivityCurve);

        // Apply speed multipliers
        double drive, turn;
        if (isFastMode) {
            drive = curveDrive * fastSpeedMultiplier;
            turn = curveTurn * fastTurnMultiplier;
        } else {
            drive = curveDrive * slowSpeedMultiplier;
            turn = curveTurn * slowTurnMultiplier;
        }

        // Calculate left and right power (arcade drive)
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Set motor powers
        if (motor1Left != null) motor1Left.setPower(leftPower);
        if (motor2Left != null) motor2Left.setPower(leftPower);
        if (motor1Right != null) motor1Right.setPower(rightPower);
        if (motor2Right != null) motor2Right.setPower(rightPower);
    }

    /**
     * Apply deadzone to joystick input
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) return 0.0;

        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        double scaled = (magnitude - deadzone) / (1.0 - deadzone);
        return sign * scaled;
    }

    /**
     * Apply sensitivity curve to input
     */
    private double applySensitivityCurve(double value, double exponent) {
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        double curved = Math.pow(magnitude, exponent);
        return sign * curved;
    }

    // Getters
    public boolean isFastMode() { return isFastMode; }

    public double getLeftPower() {
        if (motor1Left != null) return motor1Left.getPower();
        return 0.0;
    }

    public double getRightPower() {
        if (motor1Right != null) return motor1Right.getPower();
        return 0.0;
    }
}