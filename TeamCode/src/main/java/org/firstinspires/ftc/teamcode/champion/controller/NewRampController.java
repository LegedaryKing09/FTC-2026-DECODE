package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Ramp Controller using Direct Servo Position
 *
 * POSITION MAPPING:
 * - Position 0.0 = Retracted (MIN)
 * - Position 0.75 = Extended (MAX)
 *
 * PRESETS (example):
 * - FAR_RAMP_POSITION = 0.4
 * - CLOSE_RAMP_POSITION = 0.7
 * - RAMP_INCREMENT = 0.05
 */
@Config
public class NewRampController {

    public static String RAMP_SERVO_NAME = "ramp";

    // === POSITION LIMITS ===
    public static double MIN_POSITION = 0.0;    // Retracted
    public static double MAX_POSITION = 0.75;   // Extended

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
        // Clamp to valid range
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));
        initialized = true;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void update() {
        if (!initialized) {
            initialize();
        }
    }

    /**
     * Set target position (0.0 to 0.75)
     * 0.0 = Retracted, 0.75 = Extended
     */
    public void setTargetAngle(double position) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
        rampServo.setPosition(targetPosition);
    }

    /**
     * Get target position
     */
    public double getTargetAngle() {
        return targetPosition;
    }

    /**
     * Get current position (returns target since no feedback)
     */
    public double getCurrentAngle() {
        return targetPosition;
    }

    /**
     * Increment position (toward extended/0.75)
     */
    public void incrementAngle(double amount) {
        setTargetAngle(targetPosition + amount);
    }

    /**
     * Decrement position (toward retracted/0.0)
     */
    public void decrementAngle(double amount) {
        setTargetAngle(targetPosition - amount);
    }

    /**
     * Get error (always 0 since no feedback)
     */
    public double getAngleError() {
        return 0;
    }

    /**
     * Check if at target (always true for regular servo)
     */
    public boolean atTarget() {
        return true;
    }

    /**
     * Check if moving
     */
    public boolean isMoving() {
        return false;
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
        return 0;
    }

    public double getPower() {
        return 0;
    }
}