package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret Controller for TWO Position Servos (No Analog Feedback)
 *
 * HOW IT WORKS:
 * - Two servos drive the turret (same direction, same position)
 * - Position is tracked via commanded servo value (no analog wire)
 * - ZERO_POSITION is the servo value where turret is physically at 0 degrees
 *   (find this using TurretZeroFinder opmode)
 * - getTurretAngle() returns the COMMANDED angle (no analog feedback)
 * - Gear ratio is 1:1
 *
 * SETUP:
 * 1. Run TurretZeroFinder to find your ZERO_POSITION value
 * 2. Set ZERO_POSITION to that value
 * 3. Use setAngle() to command turret to a target angle
 * 4. Use setPower() for manual control (simulated via position stepping)
 *
 * AUTON → TELEOP TRANSFER:
 *   PoseStorage.turretAngle = turret.getTurretAngle();  // end of auton
 *   turret.restoreAngle(PoseStorage.turretAngle);       // start of teleop
 */
@Config
public class TurretController {

    // Hardware names
    public static String SERVO1_NAME = "turret1";
    public static String SERVO2_NAME = "turret2";

    // Zero calibration – find this with TurretZeroFinder
    // 0.5 = 0 degrees (field forward)
    public static double ZERO_POSITION = 0.5;

    // Servo range – full 0-to-1 sweep covers 315 degrees
    public static double SERVO_RANGE_DEG = 315.0;

    // Direction
    public static boolean INVERT = false;

    // Max turret angle from center (derived: ZERO_POSITION * SERVO_RANGE_DEG)
    // With ZERO_POSITION=0.5 and 315° range: ±157.5°
    // Turret won't exceed these limits due to servo 0-1 clamping

    // Power simulation – how fast setPower() moves the servo per update cycle
    public static double POWER_STEP_SCALE = 0.01;

    // Hardware
    private final Servo servo1;
    private final Servo servo2;

    // State
    private double commandedPosition;  // Last commanded servo position (0-1)
    private boolean initialized = false;
    private double currentPower = 0;

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
        servo1 = hardwareMap.get(Servo.class, SERVO1_NAME);
        servo2 = hardwareMap.get(Servo.class, SERVO2_NAME);
    }

    /**
     * Initialize – moves servo to ZERO_POSITION (turret angle = 0)
     */
    public void initialize() {
        commandedPosition = ZERO_POSITION;
        setServos(commandedPosition);
        initialized = true;
        currentPower = 0;
    }

    /**
     * Restore angle from auton – sets servo to the position corresponding
     * to the saved turret angle
     */
    public void restoreAngle(double savedAngle) {
        commandedPosition = angleToServoPosition(savedAngle);
        setServos(commandedPosition);
        initialized = true;
    }

    /**
     * Reset turret angle to 0 at current commanded position
     */
    public void resetAngle() {
        ZERO_POSITION = commandedPosition;
    }

    /**
     * Update – call every loop.
     * When using setPower(), this moves the servo position incrementally.
     */
    public void update() {
        if (!initialized) {
            initialize();
            return;
        }

        if (Math.abs(currentPower) > 0.01) {
            double step = currentPower * POWER_STEP_SCALE;
            if (INVERT) step = -step;

            commandedPosition += step;
            commandedPosition = Math.max(0.0, Math.min(1.0, commandedPosition));
            setServos(commandedPosition);
        }
    }

    /**
     * Get turret angle from commanded position (no analog feedback)
     * Returns degrees relative to ZERO_POSITION
     */
    public double getTurretAngle() {
        if (!initialized) initialize();

        double offsetFromZero = commandedPosition - ZERO_POSITION;
        double angle = offsetFromZero * SERVO_RANGE_DEG;

        if (INVERT) angle = -angle;
        return angle;
    }

    /**
     * Get turret angle normalized to -180 to +180
     */
    public double getTurretAngleNormalized() {
        double angle = getTurretAngle();
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Set turret to a specific angle (degrees relative to zero)
     */
    public void setAngle(double angleDeg) {
        currentPower = 0;
        commandedPosition = angleToServoPosition(angleDeg);
        commandedPosition = Math.max(0.0, Math.min(1.0, commandedPosition));
        setServos(commandedPosition);
    }

    /**
     * Set servo power (-1.0 to 1.0)
     * Simulates CR behavior – update() steps the position each loop
     */
    public void setPower(double power) {
        currentPower = power;
    }

    /**
     * Stop the turret (zero power, holds current position)
     */
    public void stop() {
        currentPower = 0;
    }

    /**
     * Set servo position directly (0-1)
     */
    public void setServoPosition(double position) {
        currentPower = 0;
        commandedPosition = Math.max(0.0, Math.min(1.0, position));
        setServos(commandedPosition);
    }

    /**
     * Get last commanded servo position (0-1)
     */
    public double getCommandedPosition() {
        return commandedPosition;
    }

    /**
     * Get current power
     */
    public double getPower() {
        return currentPower;
    }

    /**
     * Check if initialized
     */
    public boolean isInitialized() {
        return initialized;
    }

    // === INTERNAL ===

    /**
     * Set both servos to the same position
     */
    private void setServos(double position) {
        servo1.setPosition(position);
        servo2.setPosition(position);
    }

    /**
     * Convert a turret angle (degrees) to a servo position (0-1)
     */
    private double angleToServoPosition(double angleDeg) {
        double offset = angleDeg / SERVO_RANGE_DEG;
        if (INVERT) offset = -offset;
        return ZERO_POSITION + offset;
    }
}