package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret Controller for Axon Mini servo with 2.5:1 gear ratio
 *
 * PROBLEM SOLVED:
 * - Axon servo analog reads 0-360° for the SERVO
 * - With 2.5:1 gear ratio, turret only moves 144° per servo rotation
 * - Servo analog resets every 360° (servo) = 144° (turret)
 * - This controller tracks accumulated rotation to give continuous 0-360° turret reading
 *
 * HOW IT WORKS:
 * 1. At startup, we record the initial servo angle as "zero reference"
 * 2. Each update, we detect when servo crosses 0°/360° boundary (wraparound)
 * 3. We count full servo rotations and add them to get total turret angle
 * 4. Final turret angle = (servo rotations * 144°) + (current servo segment / gear ratio)
 *
 * EXAMPLE:
 * - Turret starts at some position → we call this 0°
 * - Turret rotates 200° clockwise
 * - Servo has done 200° * 2.5 = 500° = 1 full rotation + 140°
 * - We track: 1 rotation (144° turret) + (140° / 2.5) = 144° + 56° = 200° ✓
 */
@Config
public class TurretController {

    // Hardware names
    public static String SERVO_NAME = "turret";
    public static String ANALOG_NAME = "turret_analog";

    // Configuration
    public static double VOLTAGE_MAX = 3.3;
    public static double GEAR_RATIO = 2.5;
    public static double TURRET_DEGREES_PER_SERVO_ROTATION = 360.0 / GEAR_RATIO;

    // Hardware
    private final CRServo servo;
    private final AnalogInput analog;

    // === ANGLE TRACKING STATE ===
    private double zeroServoAngle = 0;      // Servo angle at startup (our "zero" reference)
    private double lastServoAngle = 0;      // Previous servo angle (for wraparound detection)
    private int servoRotationCount = 0;     // How many full servo rotations from start
    private boolean initialized = false;

    /**
     * Constructor using LinearOpMode (backwards compatible)
     */
    public TurretController(LinearOpMode opMode) {
        this(opMode.hardwareMap);
    }

    /**
     * Constructor using HardwareMap directly
     */
    public TurretController(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        analog = hardwareMap.get(AnalogInput.class, ANALOG_NAME);
    }

    /**
     * Initialize the turret - MUST call this at startup!
     * Sets current position as 0° reference
     */
    public void initialize() {
        zeroServoAngle = getRawServoAngle();
        lastServoAngle = zeroServoAngle;
        servoRotationCount = 0;
        initialized = true;
    }

    /**
     * Reset turret angle to 0 at current position
     */
    public void resetAngle() {
        initialize();
    }

    /**
     * Update angle tracking - MUST call this every loop!
     * Detects wraparound and updates rotation count
     */
    public void update() {
        if (!initialized) {
            initialize();
            return;
        }

        double currentServoAngle = getRawServoAngle();

        // Detect wraparound (servo crossing 0°/360° boundary)
        double delta = currentServoAngle - lastServoAngle;

        // If delta is large negative, servo wrapped from ~360° to ~0° (turning positive/CW)
        if (delta < -180) {
            servoRotationCount++;
        }
        // If delta is large positive, servo wrapped from ~0° to ~360° (turning negative/CCW)
        else if (delta > 180) {
            servoRotationCount--;
        }

        lastServoAngle = currentServoAngle;
    }

    /**
     * Get turret angle in degrees (0-360° for full turret rotation)
     * Continuous tracking - no resets at 144°
     *
     * @return Turret angle in degrees (0 = starting position)
     */
    public double getTurretAngle() {
        if (!initialized) {
            initialize();
        }

        double currentServoAngle = getRawServoAngle();

        // Calculate servo angle relative to start (accounting for wraparound)
        double servoAngleFromStart = (servoRotationCount * 360.0) + (currentServoAngle - zeroServoAngle);

        // Convert servo angle to turret angle using gear ratio
        double turretAngle = servoAngleFromStart / GEAR_RATIO;

        return turretAngle;
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
     * Positive = clockwise (turret angle increases)
     * Negative = counter-clockwise (turret angle decreases)
     */
    public void setPower(double power) {
        servo.setPower(power);
    }

    /**
     * Stop the turret
     */
    public void stop() {
        servo.setPower(0);
    }

    /**
     * Get raw servo angle (0-360°) directly from analog
     * This resets every 360° of SERVO rotation
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
     * Get current servo power
     */
    public double getPower() {
        return servo.getPower();
    }

    /**
     * Get servo rotation count (for debugging)
     */
    public int getServoRotationCount() {
        return servoRotationCount;
    }

    /**
     * Get the zero reference angle (for debugging)
     */
    public double getZeroReference() {
        return zeroServoAngle;
    }

    /**
     * Check if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }
}