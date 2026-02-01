package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret Controller with Continuous Angle Tracking
 *
 * Tracks turret angle using analog encoder with wraparound detection.
 * Supports saving/restoring angle for auton→teleop transfer.
 *
 * AUTON → TELEOP TRANSFER:
 * In Auton (at end):
 *   PoseStorage.turretAngle = turret.getTurretAngle();
 *
 * In Teleop (at start):
 *   turret.restoreAngle(PoseStorage.turretAngle);
 */
@Config
public class TurretController {

    // Hardware names
    public static String SERVO_NAME = "turret";
    public static String ANALOG_NAME = "turret_analog";

    // Configuration
    public static double VOLTAGE_MAX = 3.3;
    public static double GEAR_RATIO = 2.5;  // Servo turns 2.5x for every 1x turret turn

    // Servo direction
    public static boolean INVERT_SERVO = false;

    // Hardware
    private final CRServo servo;
    private final AnalogInput analog;

    // === ANGLE TRACKING STATE ===
    private double lastServoAngle = 0;          // Previous servo angle (for wraparound detection)
    private double accumulatedTurretAngle = 0;  // Total turret angle (continuous)
    private boolean initialized = false;

    /**
     * Constructor using LinearOpMode
     */
    public TurretController(LinearOpMode opMode) {
        this(opMode.hardwareMap);
    }

    /**
     * Constructor using HardwareMap
     */
    public TurretController(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        analog = hardwareMap.get(AnalogInput.class, ANALOG_NAME);
    }

    /**
     * Initialize - sets current position as 0°
     * Call at start of auton
     */
    public void initialize() {
        lastServoAngle = getRawServoAngle();
        accumulatedTurretAngle = 0;
        initialized = true;
    }

    /**
     * Restore angle from auton - sets current position as the saved angle
     * Call at start of teleop instead of initialize()
     *
     * @param savedAngle The turret angle saved from auton
     */
    public void restoreAngle(double savedAngle) {
        lastServoAngle = getRawServoAngle();
        accumulatedTurretAngle = savedAngle;
        initialized = true;
    }

    /**
     * Update angle tracking - call every loop!
     */
    public void update() {
        if (!initialized) {
            initialize();
            return;
        }

        double currentServoAngle = getRawServoAngle();

        // Calculate delta with wraparound handling
        double delta = currentServoAngle - lastServoAngle;

        // Handle wraparound (servo crosses 0°/360° boundary)
        if (delta < -180) {
            delta += 360;  // Wrapped forward (e.g., 350° → 10°)
        } else if (delta > 180) {
            delta -= 360;  // Wrapped backward (e.g., 10° → 350°)
        }

        // Accumulate turret angle (servo movement / gear ratio)
        accumulatedTurretAngle += delta / GEAR_RATIO;

        lastServoAngle = currentServoAngle;
    }

    /**
     * Get raw servo angle (0-360°) from analog sensor
     */
    public double getRawServoAngle() {
        double voltage = analog.getVoltage();
        return (voltage / VOLTAGE_MAX) * 360.0;
    }

    /**
     * Get continuous turret angle in degrees
     */
    public double getTurretAngle() {
        return accumulatedTurretAngle;
    }

    /**
     * Set turret power (-1 to +1)
     */
    public void setPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        servo.setPower(INVERT_SERVO ? -power : power);
    }

    /**
     * Stop the turret
     */
    public void stop() {
        servo.setPower(0);
    }

    /**
     * Check if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Reset angle to 0 at current position
     */
    public void resetAngle() {
        initialize();
    }

    /**
     * Get raw voltage (for debugging)
     */
    public double getVoltage() {
        return analog.getVoltage();
    }
}