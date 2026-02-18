package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret Controller for TWO Position Servos with Field-Centric Auto-Aim
 *
 * SERVO MAPPING:
 * - Servo 0.0 to 1.0 = 315° total range
 * - Servo 0.5 = 0° (field forward)
 * - Servo 0.0 = -157.5°, Servo 1.0 = +157.5°
 *
 * AUTO-AIM MODES:
 * 1. Fixed field angle: setFieldAngle(angle) then updateAutoAim(heading) every loop
 *    - Turret holds a fixed direction on the field, compensating for robot rotation
 * 2. Target tracking: setTarget(x, y) then updateAutoAim(robotX, robotY, heading) every loop
 *    - Turret continuously aims at a field coordinate
 *
 * MANUAL CONTROL:
 * - setServoPosition(pos): direct servo command (0-1)
 * - setPower(power): simulated CR via position stepping (call update() each loop)
 * - nudge(step): bump servo position by a small amount
 *
 * USAGE IN TELEOP:
 *   turret.enableAutoAim();
 *   turret.setTarget(0, 0);  // aim at origin
 *   // in loop:
 *   turret.updateAutoAim(drive.getX(), drive.getY(), drive.getHeadingDegrees());
 *
 * USAGE IN AUTON:
 *   turret.enableAutoAim();
 *   turret.setFieldAngle(-45);  // fixed field angle
 *   // in loop:
 *   turret.updateAutoAim(headingDeg);  // heading-only compensation
 */
@Config
public class TurretController {

    // Hardware names
    public static String SERVO1_NAME = "turret1";
    public static String SERVO2_NAME = "turret2";

    // Zero calibration – 0.5 = 0 degrees (field forward)
    public static double ZERO_POSITION = 0.51;

    // Servo range – full 0-to-1 sweep covers 315 degrees
    public static double SERVO_RANGE_DEG = 315.0;

    // Direction
    public static boolean INVERT = false;

    // Invert heading compensation (set true if yaw scalar is negative)
    public static boolean INVERT_HEADING = true;

    // Turret pivot offset from robot center (inches, in robot-local frame)
    // FORWARD_OFFSET = positive means turret is in front of robot center
    // RIGHT_OFFSET = positive means turret is to the right of robot center
    public static double TURRET_FORWARD_OFFSET = -4.0;
    public static double TURRET_RIGHT_OFFSET = 0.0;

    // Power simulation – how fast setPower() moves the servo per update cycle
    public static double POWER_STEP_SCALE = 0.008;

    // Hardware
    private final Servo servo1;
    private final Servo servo2;

    // Servo state
    private double commandedPosition;
    private boolean initialized = false;
    private double currentPower = 0;

    // Auto-aim state
    private boolean autoAimEnabled = false;
    private double targetFieldAngle = 0;
    private double targetX = 0;
    private double targetY = 0;
    private boolean useTargetTracking = false;
    private double aimOffset = 0;

    // Telemetry-accessible state
    private double lastTurretRelativeAngle = 0;
    private double lastTargetFieldAngle = 0;

    // =========================================================================
    // CONSTRUCTORS
    // =========================================================================

    public TurretController(LinearOpMode opMode) {
        this(opMode.hardwareMap);
    }

    public TurretController(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(Servo.class, SERVO1_NAME);
        servo2 = hardwareMap.get(Servo.class, SERVO2_NAME);
    }

    // =========================================================================
    // AUTO-AIM API
    // =========================================================================

    /** Enable auto-aim. Call updateAutoAim() every loop after enabling. */
    public void enableAutoAim() {
        autoAimEnabled = true;
    }

    /** Disable auto-aim. Turret holds current position. */
    public void disableAutoAim() {
        autoAimEnabled = false;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    /**
     * Set a fixed field angle for auto-aim to hold.
     * Robot heading compensation keeps turret pointing this direction.
     */
    public void setFieldAngle(double fieldAngleDeg) {
        targetFieldAngle = fieldAngleDeg;
        useTargetTracking = false;
    }

    /**
     * Set a target field position to continuously aim at.
     * Requires robotX, robotY in updateAutoAim() calls.
     */
    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
        useTargetTracking = true;
    }

    /** Set driver aim offset (added to calculated angle). */
    public void setAimOffset(double offsetDeg) {
        aimOffset = offsetDeg;
    }

    /** Adjust aim offset by a delta. */
    public void addAimOffset(double deltaDeg) {
        aimOffset += deltaDeg;
    }

    public double getAimOffset() {
        return aimOffset;
    }

    /**
     * Update auto-aim — fixed field angle mode (heading compensation only).
     * Call every loop when using setFieldAngle().
     *
     * @param robotHeadingDeg current robot heading in degrees
     */
    public void updateAutoAim(double robotHeadingDeg) {
        if (!autoAimEnabled) return;

        double fieldAngle = targetFieldAngle + aimOffset;
        lastTargetFieldAngle = fieldAngle;

        double servoPos = fieldAngleToServoPosition(fieldAngle, robotHeadingDeg);
        setServoPositionInternal(servoPos);
    }

    /**
     * Update auto-aim — full position tracking mode.
     * Call every loop when using setTarget().
     * Also works for fixed field angle (ignores robotX/Y in that case).
     *
     * Compensates for turret pivot offset from robot center:
     * Converts TURRET_FORWARD_OFFSET and TURRET_RIGHT_OFFSET from robot-local
     * frame to field frame using robot heading, then calculates angle to target
     * from the turret's actual field position (not robot center).
     *
     * @param robotX robot X position on field (odometry center)
     * @param robotY robot Y position on field (odometry center)
     * @param robotHeadingDeg robot heading in degrees
     */
    public void updateAutoAim(double robotX, double robotY, double robotHeadingDeg) {
        if (!autoAimEnabled) return;

        // Calculate turret's actual field position by rotating the offset by heading
        double headingRad = Math.toRadians(robotHeadingDeg);
        double turretFieldX = robotX
                + TURRET_FORWARD_OFFSET * Math.sin(headingRad)
                + TURRET_RIGHT_OFFSET * Math.cos(headingRad);
        double turretFieldY = robotY
                + TURRET_FORWARD_OFFSET * Math.cos(headingRad)
                - TURRET_RIGHT_OFFSET * Math.sin(headingRad);

        double fieldAngle;
        if (useTargetTracking) {
            fieldAngle = calculateFieldAngleToTarget(turretFieldX, turretFieldY) + aimOffset;
        } else {
            fieldAngle = targetFieldAngle + aimOffset;
        }
        lastTargetFieldAngle = fieldAngle;

        double servoPos = fieldAngleToServoPosition(fieldAngle, robotHeadingDeg);
        setServoPositionInternal(servoPos);
    }

    // =========================================================================
    // MANUAL CONTROL
    // =========================================================================

    /** Initialize – moves servo to ZERO_POSITION (turret angle = 0) */
    public void initialize() {
        commandedPosition = ZERO_POSITION;
        setServos(commandedPosition);
        initialized = true;
        currentPower = 0;
    }

    /** Restore angle from auton */
    public void restoreAngle(double savedAngle) {
        commandedPosition = angleToServoPosition(savedAngle);
        setServos(commandedPosition);
        initialized = true;
    }

    /** Reset turret angle to 0 at current commanded position */
    public void resetAngle() {
        ZERO_POSITION = commandedPosition;
    }

    /**
     * Update – call every loop for setPower() stepping.
     * Do NOT call when auto-aim is enabled (they conflict).
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

    /** Set servo position directly (0-1). Disables power stepping. */
    public void setServoPosition(double position) {
        currentPower = 0;
        commandedPosition = Math.max(0.0, Math.min(1.0, position));
        setServos(commandedPosition);
    }

    /** Nudge servo position by a step amount. */
    public void nudge(double step) {
        currentPower = 0;
        commandedPosition = Math.max(0.0, Math.min(1.0, commandedPosition + step));
        setServos(commandedPosition);
    }

    /** Set turret to a specific angle (degrees relative to zero) */
    public void setAngle(double angleDeg) {
        currentPower = 0;
        commandedPosition = angleToServoPosition(angleDeg);
        commandedPosition = Math.max(0.0, Math.min(1.0, commandedPosition));
        setServos(commandedPosition);
    }

    /** Set servo power (-1.0 to 1.0). update() steps the position each loop. */
    public void setPower(double power) {
        currentPower = power;
    }

    /** Stop the turret (zero power, holds current position) */
    public void stop() {
        currentPower = 0;
    }

    // =========================================================================
    // GETTERS
    // =========================================================================

    public double getTurretAngle() {
        if (!initialized) initialize();
        double offsetFromZero = commandedPosition - ZERO_POSITION;
        double angle = offsetFromZero * SERVO_RANGE_DEG;
        if (INVERT) angle = -angle;
        return angle;
    }

    public double getTurretAngleNormalized() {
        double angle = getTurretAngle();
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public double getCommandedPosition() {
        return commandedPosition;
    }

    public double getPower() {
        return currentPower;
    }

    public boolean isInitialized() {
        return initialized;
    }

    /** Last computed turret-relative angle (for telemetry) */
    public double getLastTurretRelativeAngle() {
        return lastTurretRelativeAngle;
    }

    /** Last computed target field angle including offset (for telemetry) */
    public double getLastTargetFieldAngle() {
        return lastTargetFieldAngle;
    }

    // =========================================================================
    // INTERNAL: Auto-aim math
    // =========================================================================

    private double calculateFieldAngleToTarget(double robotX, double robotY) {
        double dx = targetX - robotX;
        double dy = targetY - robotY;

        if (Math.abs(dy) < 0.001 && Math.abs(dx) < 0.001) {
            return 0;
        }

        return Math.toDegrees(Math.atan2(-dx, -dy));
    }

    private double fieldAngleToServoPosition(double fieldAngleDeg, double robotHeadingDeg) {
        double heading = INVERT_HEADING ? -robotHeadingDeg : robotHeadingDeg;
        double turretAngle = fieldAngleDeg - heading;

        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        lastTurretRelativeAngle = turretAngle;

        double servoPos = ZERO_POSITION + (-turretAngle / SERVO_RANGE_DEG);
        return Math.max(0.0, Math.min(1.0, servoPos));
    }

    private void setServoPositionInternal(double position) {
        commandedPosition = Math.max(0.0, Math.min(1.0, position));
        setServos(commandedPosition);
    }

    // =========================================================================
    // INTERNAL: Servo hardware
    // =========================================================================

    private void setServos(double position) {
        servo1.setPosition(position);
        servo2.setPosition(position);
    }

    private double angleToServoPosition(double angleDeg) {
        double offset = angleDeg / SERVO_RANGE_DEG;
        if (INVERT) offset = -offset;
        return ZERO_POSITION + offset;
    }
}