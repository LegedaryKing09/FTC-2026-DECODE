package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret Controller for TWO Axon Mini servos with 2.5:1 gear ratio
 *
 * HOW IT WORKS:
 * 1. Each update, calculate servo delta from last reading
 * 2. Handle wraparound: if servo jumps from 359° to 5°, delta should be +6°, not -354°
 * 3. Convert servo delta to turret delta: turretDelta = servoDelta / gearRatio
 * 4. Accumulate: turretAngle += turretDelta
 *
 * DUAL SERVO SETUP:
 * - Both servos drive the turret together for more torque
 * - servoLeft and servoRight can be inverted independently
 * - Only one analog encoder is needed for position feedback
 */
@Config
public class TurretController {

    // Hardware names
    public static String SERVO_LEFT_NAME = "turret1";
    public static String SERVO_RIGHT_NAME = "turret2";
    public static String ANALOG_NAME = "turret_analog";

    // Configuration
    public static double VOLTAGE_MAX = 3.3;
    public static double GEAR_RATIO = 2.5;

    // Servo direction control (adjust based on mounting)
    public static boolean INVERT_LEFT = false;
    public static boolean INVERT_RIGHT = true;  // Usually mirrored

    // Hardware
    private final CRServo servoLeft;
    private final CRServo servoRight;
    private final AnalogInput analog;

    // === ANGLE TRACKING STATE ===
    private double lastServoAngle = 0;          // Previous servo angle
    private double accumulatedTurretAngle = 0;  // The turret angle we're tracking
    private int servoRotationCount = 0;         // For debugging
    private boolean initialized = false;
    private double currentPower = 0;            // Track current power

    /**
     * Constructor using LinearOpMode
     */
    public TurretController(LinearOpMode opMode) {
        this(opMode.hardwareMap);
    }

    /**
     * Constructor using HardwareMap directly
     */
    public TurretController(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(CRServo.class, SERVO_LEFT_NAME);
        servoRight = hardwareMap.get(CRServo.class, SERVO_RIGHT_NAME);
        analog = hardwareMap.get(AnalogInput.class, ANALOG_NAME);
    }

    /**
     * Initialize the turret - MUST call this at startup!
     * Sets current position as 0° reference
     */
    public void initialize() {
        lastServoAngle = getRawServoAngle();
        accumulatedTurretAngle = 0;
        servoRotationCount = 0;
        initialized = true;
        stop();
    }

    /**
     * Reset turret angle to 0 at current position
     */
    public void resetAngle() {
        initialize();
    }

    /**
     * Update angle tracking - MUST call this every loop!
     * Calculates delta, handles wraparound, accumulates turret angle
     */
    public void update() {
        if (!initialized) {
            initialize();
            return;
        }

        double currentServoAngle = getRawServoAngle();

        // Calculate raw delta
        double servoDelta = currentServoAngle - lastServoAngle;

        // Handle wraparound
        // If delta is very negative (like -354), servo wrapped forward (359° → 5°)
        // Actual delta should be positive: -354 + 360 = +6°
        if (servoDelta < -180) {
            servoDelta += 360;
            servoRotationCount++;
        }
        // If delta is very positive (like +354), servo wrapped backward (5° → 359°)
        // Actual delta should be negative: +354 - 360 = -6°
        else if (servoDelta > 180) {
            servoDelta -= 360;
            servoRotationCount--;
        }

        // Convert servo delta to turret delta and accumulate
        double turretDelta = servoDelta / GEAR_RATIO;
        accumulatedTurretAngle += turretDelta;

        // Save for next iteration
        lastServoAngle = currentServoAngle;
    }

    /**
     * Get turret angle in degrees
     * 0° = starting position
     */
    public double getTurretAngle() {
        if (!initialized) {
            initialize();
        }
        return accumulatedTurretAngle;
    }

    /**
     * Get turret angle normalized to -180° to +180°
     */
    public double getTurretAngleNormalized() {
        double angle = getTurretAngle();
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Set servo power directly (-1.0 to 1.0)
     * Both servos receive the same command (with inversion applied)
     */
    public void setPower(double power) {
        currentPower = power;

        double leftPower = INVERT_LEFT ? -power : power;
        double rightPower = INVERT_RIGHT ? -power : power;

        servoLeft.setPower(leftPower);
        servoRight.setPower(rightPower);
    }

    /**
     * Stop the turret
     */
    public void stop() {
        currentPower = 0;
        servoLeft.setPower(0);
        servoRight.setPower(0);
    }

    /**
     * Get raw servo angle (0-360°) directly from analog
     */
    public double getRawServoAngle() {
        double voltage = analog.getVoltage();
        return (voltage / VOLTAGE_MAX) * 360.0;
    }

    /**
     * Get raw analog voltage (for debugging)
     */
    public double getVoltage() {
        return analog.getVoltage();
    }

    /**
     * Get current servo power (commanded, not actual)
     */
    public double getPower() {
        return currentPower;
    }

    /**
     * Get servo rotation count (for debugging)
     */
    public int getServoRotationCount() {
        return servoRotationCount;
    }

    /**
     * Check if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }
}