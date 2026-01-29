package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Field-Centric Turret Controller with Dead Zone Handling
 *
 * TURRET RANGE:
 * - From 0°, can go CCW (positive) up to CCW_LIMIT (default 90°)
 * - From 0°, can go CW (negative) down to CW_LIMIT (default -90°)
 * - Dead zone is between CCW_LIMIT and CW_LIMIT (going the long way)
 *
 * DEAD ZONE BEHAVIOR:
 * - When target would require entering dead zone, turret STOPS at limit
 * - Turret resumes tracking when target moves back into reachable range
 * - No automatic unwrapping - turret simply waits at the edge
 *
 * DIRECTION-DEPENDENT CONTROL:
 * - CCW (negative error): Uses PID
 * - CW (positive error): Uses PIDF with feedforward
 */
@Config
public class TurretFieldController {

    private final TurretController turret;
    private final FtcDashboard dashboard;

    // ========== FIELD TARGET ==========
    public static double TARGET_FIELD_ANGLE = 27.0;

    // ========== TURRET LIMITS (Dead Zone) ==========
    // From 0°, turret can go:
    // - CCW (positive direction): up to CCW_LIMIT
    // - CW (negative direction): down to CW_LIMIT
    public static double CCW_LIMIT = 180.0;    // Max CCW position
    public static double CW_LIMIT = -90.0;    // Max CW position (negative)
    public static boolean USE_LIMITS = true;

    // ========== CCW PID (negative error - the smooth one!) ==========
    public static double CCW_kP = 0.06;
    public static double CCW_kI = 0.0;
    public static double CCW_kD = 0.005;

    // ========== CW PIDF (positive error - with feedforward) ==========
    public static double CW_kP = 0.05;
    public static double CW_kI = 0.0;
    public static double CW_kD = 0.003;
    public static double CW_kF = 0.015;

    // ========== POWER LIMITS ==========
    public static double MAX_POWER = 0.7;
    public static double MIN_POWER = 0.06;
    public static double INTEGRAL_LIMIT = 15.0;

    // ========== ALIGNMENT ==========
    public static double FIELD_TOLERANCE_DEG = 1.5;

    // ========== OUTPUT ==========
    public static boolean INVERT_OUTPUT = true;

    // ========== OFFSET ==========
    public static double TURRET_OFFSET = 0.0;

    // ========== DASHBOARD ==========
    public static boolean ENABLE_GRAPHING = true;

    // ========== STATE ==========
    private boolean enabled = false;
    private boolean inDeadZone = false;  // True when target is unreachable
    private double lastRobotHeading = 0.0;
    private double lastFieldError = 0.0;
    private double lastPower = 0.0;

    // PID state
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private long lastUpdateTime = 0;

    // For telemetry
    private double lastPTerm = 0.0;
    private double lastITerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastFTerm = 0.0;

    public TurretFieldController(TurretController turret) {
        this.turret = turret;
        this.dashboard = FtcDashboard.getInstance();
    }

    public void setTargetFieldAngle(double fieldAngleDegrees) {
        TARGET_FIELD_ANGLE = fieldAngleDegrees;
    }

    public double getTargetFieldAngle() {
        return TARGET_FIELD_ANGLE;
    }

    public void enable() {
        enabled = true;
        inDeadZone = false;
        resetPID();
    }

    public void disable() {
        enabled = false;
        inDeadZone = false;
        turret.stop();
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Check if target is currently in dead zone (unreachable)
     */
    public boolean isInDeadZone() {
        return inDeadZone;
    }

    public void resetPID() {
        lastError = 0.0;
        integralSum = 0.0;
        lastUpdateTime = 0;
        lastPTerm = 0.0;
        lastITerm = 0.0;
        lastDTerm = 0.0;
        lastFTerm = 0.0;
    }

    /**
     * Main update - call every loop!
     * Call turret.update() BEFORE this!
     */
    public double update(double robotHeadingDegrees) {
        if (!enabled) {
            lastPower = 0.0;
            sendGraphData(0, 0, 0, 0, 0, 0, false);
            return 0.0;
        }

        // Time delta
        long currentTime = System.currentTimeMillis();
        double dt = (lastUpdateTime == 0) ? 0.02 : (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;
        dt = Math.max(0.005, Math.min(0.1, dt));

        // Store previous heading for feedforward
        double prevRobotHeading = lastRobotHeading;
        lastRobotHeading = robotHeadingDegrees;

        double currentTurretAngle = turret.getTurretAngle();

        // Calculate where turret needs to be to face field target
        double requiredTurretAngle = TARGET_FIELD_ANGLE - robotHeadingDegrees - TURRET_OFFSET;

        // Normalize to find shortest path
        requiredTurretAngle = normalizeToRange(requiredTurretAngle, currentTurretAngle);

        // ========== DEAD ZONE CHECK ==========
        if (USE_LIMITS) {
            if (requiredTurretAngle > CCW_LIMIT) {
                // Target is in dead zone (past CCW limit)
                inDeadZone = true;
                // Stop at CCW limit
                requiredTurretAngle = CCW_LIMIT;
            } else if (requiredTurretAngle < CW_LIMIT) {
                // Target is in dead zone (past CW limit)
                inDeadZone = true;
                // Stop at CW limit
                requiredTurretAngle = CW_LIMIT;
            } else {
                // Target is reachable
                inDeadZone = false;
            }
        } else {
            inDeadZone = false;
        }

        // Calculate error (how far turret needs to move)
        double error = requiredTurretAngle - currentTurretAngle;
        lastFieldError = error;

        // If in dead zone and at limit, stop
        if (inDeadZone) {
            // Check if we're already at the limit (within tolerance)
            boolean atCCWLimit = Math.abs(currentTurretAngle - CCW_LIMIT) < 3.0;
            boolean atCWLimit = Math.abs(currentTurretAngle - CW_LIMIT) < 3.0;

            if ((requiredTurretAngle >= CCW_LIMIT && atCCWLimit) ||
                    (requiredTurretAngle <= CW_LIMIT && atCWLimit)) {
                // At limit, stop and wait
                turret.stop();
                lastPower = 0.0;
                resetPID();
                sendGraphData(error, 0, 0, 0, 0, 0, true);
                return 0.0;
            }
            // Otherwise, continue moving toward the limit
        }

        // Calculate robot rotation rate for feedforward
        double robotRotationRate = normalizeAngle(robotHeadingDegrees - prevRobotHeading) / dt;

        // ========== DIRECTION-DEPENDENT CONTROL ==========
        double output;
        if (error >= 0) {
            // CW direction (positive error) - use PIDF
            output = calculateCW_PIDF(error, robotRotationRate, dt);
        } else {
            // CCW direction (negative error) - use PID only
            output = calculateCCW_PID(error, dt);
        }

        // Apply power limits
        double absOutput = Math.abs(output);
        if (absOutput > MAX_POWER) {
            output = Math.copySign(MAX_POWER, output);
        } else if (absOutput > 0.001 && absOutput < MIN_POWER) {
            output = Math.copySign(MIN_POWER, output);
        }

        // Apply inversion
        if (INVERT_OUTPUT) {
            output = -output;
        }

        turret.setPower(output);
        lastPower = output;

        sendGraphData(error, output, lastPTerm, lastITerm, lastDTerm, lastFTerm, inDeadZone);
        return output;
    }

    /**
     * Normalize angle to be closest to reference angle
     * This finds the equivalent angle that minimizes distance to current position
     */
    private double normalizeToRange(double angle, double reference) {
        while (angle - reference > 180) angle -= 360;
        while (angle - reference < -180) angle += 360;
        return angle;
    }

    /**
     * CW direction: PIDF with feedforward
     */
    private double calculateCW_PIDF(double error, double robotRotationRate, double dt) {
        // P term
        lastPTerm = CW_kP * error;

        // I term with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        lastITerm = CW_kI * integralSum;

        // D term
        double errorDerivative = (error - lastError) / dt;
        lastDTerm = CW_kD * errorDerivative;
        lastError = error;

        // F term - feedforward based on robot rotation
        lastFTerm = CW_kF * (-robotRotationRate);

        return lastPTerm + lastITerm + lastDTerm + lastFTerm;
    }

    /**
     * CCW direction: PID only (already smooth)
     */
    private double calculateCCW_PID(double error, double dt) {
        // P term
        lastPTerm = CCW_kP * error;

        // I term with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        lastITerm = CCW_kI * integralSum;

        // D term
        double errorDerivative = (error - lastError) / dt;
        lastDTerm = CCW_kD * errorDerivative;
        lastError = error;

        // No F term for CCW
        lastFTerm = 0.0;

        return lastPTerm + lastITerm + lastDTerm;
    }

    private void sendGraphData(double error, double power, double pTerm, double iTerm, double dTerm, double fTerm, boolean deadZone) {
        if (!ENABLE_GRAPHING) return;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret/Error", error);
        packet.put("Turret/Power", power * 100);
        packet.put("Turret/P", pTerm * 100);
        packet.put("Turret/I", iTerm * 100);
        packet.put("Turret/D", dTerm * 100);
        packet.put("Turret/F", fTerm * 100);
        packet.put("Turret/DeadZone", deadZone ? 1 : 0);
        packet.put("Turret/Dir", error >= 0 ? 1 : -1);
        dashboard.sendTelemetryPacket(packet);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public boolean isAligned() {
        // Not aligned if in dead zone
        if (inDeadZone) return false;
        return Math.abs(lastFieldError) <= FIELD_TOLERANCE_DEG;
    }

    public double getFieldError() {
        return lastFieldError;
    }

    public double getLastPower() {
        return lastPower;
    }

    public double getCurrentTurretAngle() {
        return turret.getTurretAngle();
    }

    public double getLastRobotHeading() {
        return lastRobotHeading;
    }

    // PIDF term getters for debugging
    public double getLastPTerm() { return lastPTerm; }
    public double getLastITerm() { return lastITerm; }
    public double getLastDTerm() { return lastDTerm; }
    public double getLastFTerm() { return lastFTerm; }

    // ========== AUTO-AIM FOR AUTONOMOUS ==========

    public interface HeadingProvider {
        double getHeadingDegrees();
    }

    public interface OpModeChecker {
        boolean isActive();
    }

    public static double AUTO_AIM_TIMEOUT_MS = 2000.0;

    /**
     * Auto-aim - blocks until aligned or timeout
     * Returns false immediately if target is in dead zone
     */
    public boolean autoAim(HeadingProvider headingProvider, OpModeChecker opModeChecker) {
        return autoAim(TARGET_FIELD_ANGLE, headingProvider, opModeChecker, null);
    }

    public boolean autoAim(double targetAngle, HeadingProvider headingProvider, OpModeChecker opModeChecker) {
        return autoAim(targetAngle, headingProvider, opModeChecker, null);
    }

    public boolean autoAim(double targetAngle,
                           HeadingProvider headingProvider,
                           OpModeChecker opModeChecker,
                           org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {

        setTargetFieldAngle(targetAngle);
        enable();

        long startTime = System.currentTimeMillis();
        int alignedCount = 0;

        while (opModeChecker.isActive()) {
            long elapsed = System.currentTimeMillis() - startTime;

            if (elapsed > AUTO_AIM_TIMEOUT_MS) {
                if (telemetry != null) {
                    telemetry.addData("AutoAim", "TIMEOUT");
                    telemetry.addData("DeadZone", inDeadZone);
                    telemetry.update();
                }
                disable();
                return false;
            }

            // Update
            turret.update();
            double heading = headingProvider.getHeadingDegrees();
            update(heading);

            // Telemetry
            if (telemetry != null) {
                telemetry.addData("AutoAim", "Error: %.1f°", lastFieldError);
                telemetry.addData("Turret Angle", "%.1f°", turret.getTurretAngle());
                telemetry.addData("DeadZone", inDeadZone ? "YES - STOPPED" : "No");
                telemetry.addData("Power", "%.2f", lastPower);
                telemetry.update();
            }

            // If in dead zone, return false (can't reach target)
            if (inDeadZone && Math.abs(lastPower) < 0.01) {
                if (telemetry != null) {
                    telemetry.addData("AutoAim", "TARGET IN DEAD ZONE");
                    telemetry.update();
                }
                // Don't disable - stay at limit in case target moves
                return false;
            }

            // Check aligned (need 3 consecutive)
            if (isAligned()) {
                alignedCount++;
                if (alignedCount >= 3) {
                    if (telemetry != null) {
                        telemetry.addData("AutoAim", "ALIGNED!");
                        telemetry.update();
                    }
                    return true;
                }
            } else {
                alignedCount = 0;
            }

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                disable();
                return false;
            }
        }

        disable();
        return false;
    }
}