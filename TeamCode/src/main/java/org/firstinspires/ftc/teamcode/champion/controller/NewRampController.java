package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Ramp Controller using Direct Servo Position
 *
 * API unchanged from angle-based version for easy migration.
 * "Angle" is now just servo position scaled to familiar range.
 *
 * POSITION MAPPING:
 * - "Angle" 0 = Position 0.0 = Extended
 * - "Angle" -100 = Position ~0.41
 * - "Angle" -245 = Position 1.0 = Retracted
 *
 * Just change your constants:
 * - FAR_RAMP_ANGLE = -175.0  -->  FAR_RAMP_ANGLE = 0.71
 * - CLOSE_RAMP_ANGLE = -118.4  -->  CLOSE_RAMP_ANGLE = 0.48
 * - RAMP_INCREMENT_DEGREES = 5.0  -->  RAMP_INCREMENT_DEGREES = 0.02
 */
@Config
public class NewRampController {

    public static String RAMP_SERVO_NAME = "ramp";

    // === POSITION LIMITS ===
    public static double MIN_POSITION = 0.75;   // Extended
    public static double MAX_POSITION = 0.0;   // Retracted

    private final Servo rampServo;
    private double targetPosition = 0.5;
    private boolean initialized = false;

    public NewRampController(LinearOpMode opMode) {
        this(opMode.hardwareMap);
    }

    public NewRampController(HardwareMap hardwareMap) {
        rampServo = hardwareMap.get(Servo.class, RAMP_SERVO_NAME);
    }

    /**
     * Initialize - read current position, don't move
     */
    public void initialize() {
        targetPosition = rampServo.getPosition();
        initialized = true;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void update() {
        if (!initialized) {
            initialize();
        }
        // Regular servos don't need continuous updates
    }

    /**
     * Set target "angle" - now actually servo position (0.0 to 1.0)
     *
     * MIGRATION: Change your constants from angles to positions:
     *   FAR_RAMP_ANGLE = -175.0  -->  0.71
     *   CLOSE_RAMP_ANGLE = -118.4  -->  0.48
     */
    public void setTargetAngle(double position) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
        rampServo.setPosition(targetPosition);
    }

    /**
     * Get target "angle" - returns servo position (0.0 to 1.0)
     */
    public double getTargetAngle() {
        return targetPosition;
    }

    /**
     * Get current "angle" - returns servo position (0.0 to 1.0)
     */
    public double getCurrentAngle() {
        return targetPosition;  // Servo doesn't have feedback, return target
    }

    /**
     * Increment "angle" - adds to position
     *
     * MIGRATION: Change increment from degrees to position:
     *   RAMP_INCREMENT_DEGREES = 5.0  -->  0.02
     */
    public void incrementAngle(double amount) {
        setTargetAngle(targetPosition - amount);
    }

    /**
     * Decrement "angle" - subtracts from position
     */
    public void decrementAngle(double amount) {
        setTargetAngle(targetPosition + amount);
    }

    /**
     * Get angle error - minimal since servo goes directly to position
     */
    public double getAngleError() {
        return 0;  // No feedback, assume at target
    }

    /**
     * Check if at target - always true for regular servo
     */
    public boolean atTarget() {
        return true;  // Regular servo, assume immediate
    }

    /**
     * Check if moving
     */
    public boolean isMoving() {
        return false;  // No way to know with regular servo
    }

    public void stop() {
        // Regular servos hold position automatically
    }

    /**
     * Direct servo position access
     */
    public void setServoPosition(double position) {
        setTargetAngle(position);
    }

    public double getServoPosition() {
        return rampServo.getPosition();
    }

    public double getVoltage() {
        return 0;  // No analog input in this version
    }

    public double getPower() {
        return 0;  // Regular servo, no power reading
    }
}