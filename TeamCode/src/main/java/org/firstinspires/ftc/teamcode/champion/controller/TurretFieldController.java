package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Field-Centric Turret Controller with PID + Hysteresis Deadband + Dashboard Graphing
 *
 * ANTI-JITTER FEATURE:
 * - When error < DEADBAND_ENTER (1.5°), turret STOPS and enters "locked" state
 * - Turret stays locked until error > DEADBAND_EXIT (2.5°)
 * - This prevents oscillation when robot is stationary
 *
 * DASHBOARD GRAPHING:
 * - Graphs error, power, P/I/D terms over time for PID tuning
 * - View in FTC Dashboard under "Turret PID" graph
 *
 * PID TUNING GUIDE:
 * 1. Set kI=0, kD=0, start with low kP (0.02)
 * 2. Increase kP until turret oscillates
 * 3. Reduce kP by 30-40%
 * 4. Add kD to dampen oscillation (start 0.005)
 * 5. Add small kI only if steady-state error persists
 */
@Config
public class TurretFieldController {

    private final TurretController turret;
    private final FtcDashboard dashboard;

    // ========== FIELD TARGET ==========
    public static double TARGET_FIELD_ANGLE = 35.0;

    // ========== DIRECTION-DEPENDENT PID ==========
    // Clockwise (positive error → positive power)
    public static double CW_kP = 0.05;
    public static double CW_kI = 0.0;
    public static double CW_kD = 0.0;

    // Counter-clockwise (negative error → negative power)
    public static double CCW_kP = 0.06;
    public static double CCW_kI = 0.0;
    public static double CCW_kD = 0.005;

    public static double FIELD_INTEGRAL_LIMIT = 20.0;

    public static double MAX_POWER = 0.6;
    public static double MIN_POWER = 0.08;

    // ========== HYSTERESIS DEADBAND (anti-jitter!) ==========
    // Enter locked state when error < DEADBAND_ENTER
    // Exit locked state when error > DEADBAND_EXIT
    public static double DEADBAND_ENTER = 1.5;   // Stop when within 1.5°
    public static double DEADBAND_EXIT = 2.5;    // Restart when error exceeds 2.5°

    // Legacy tolerance (for isAligned() check)
    public static double FIELD_TOLERANCE_DEG = 1.5;

    public static boolean INVERT_OUTPUT = true;

    // ========== TURRET LIMITS ==========
    public static double TURRET_MIN_ANGLE = -200.0;
    public static double TURRET_MAX_ANGLE = 200.0;
    public static boolean USE_LIMITS = true;

    // ========== WIRE SAFETY ==========
    public static double WIRE_SAFETY_THRESHOLD = 180.0;
    public static boolean USE_WIRE_SAFETY = true;

    // ========== OFFSET ==========
    public static double TURRET_OFFSET = 0.0;

    // ========== DASHBOARD GRAPHING ==========
    public static boolean ENABLE_GRAPHING = true;

    // ========== PID STATE ==========
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastUpdateTime = 0;

    // ========== STATE ==========
    private boolean enabled = false;
    private boolean isLocked = false;  // Hysteresis lock state
    private double lastRobotHeading = 0.0;
    private double calculatedTurretTarget = 0.0;
    private double lastPower = 0.0;

    // For graphing
    private double lastPTerm = 0.0;
    private double lastITerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastFieldError = 0.0;

    // Wire safety unwrapping state
    private boolean isUnwrapping = false;
    private double unwrapTarget = 0.0;

    public TurretFieldController(TurretController turret) {
        this.turret = turret;
        this.dashboard = FtcDashboard.getInstance();
    }

    /**
     * Set the field angle the turret should face
     */
    public void setTargetFieldAngle(double fieldAngleDegrees) {
        TARGET_FIELD_ANGLE = fieldAngleDegrees;
    }

    public double getTargetFieldAngle() {
        return TARGET_FIELD_ANGLE;
    }

    /**
     * Enable field-centric control
     */
    public void enable() {
        enabled = true;
        isLocked = false;
        resetPID();
    }

    /**
     * Disable field-centric control
     */
    public void disable() {
        enabled = false;
        isUnwrapping = false;
        isLocked = false;
        turret.stop();
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Check if turret is currently locked (within deadband)
     */
    public boolean isLocked() {
        return isLocked;
    }

    /**
     * Reset PID state (does NOT reset turret angle!)
     */
    public void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
        lastUpdateTime = 0;
        lastPTerm = 0.0;
        lastITerm = 0.0;
        lastDTerm = 0.0;
    }

    /**
     * Update the controller - call every loop!
     * Make sure to call turret.update() BEFORE this!
     *
     * @param robotHeadingDegrees Current robot heading from IMU
     * @return The turret power being applied
     */
    public double update(double robotHeadingDegrees) {
        if (!enabled) {
            lastPower = 0.0;
            sendGraphData(0, 0, 0, 0, 0);
            return 0.0;
        }

        lastRobotHeading = robotHeadingDegrees;

        // Get current turret angle (absolute, never reset during operation)
        double currentTurretAngle = turret.getTurretAngle();

        // ========== WIRE SAFETY CHECK ==========
        if (USE_WIRE_SAFETY && !isUnwrapping) {
            if (Math.abs(currentTurretAngle) > WIRE_SAFETY_THRESHOLD) {
                isUnwrapping = true;
                if (currentTurretAngle > WIRE_SAFETY_THRESHOLD) {
                    unwrapTarget = currentTurretAngle - 225.0;
                } else {
                    unwrapTarget = currentTurretAngle + 225.0;
                }
            }
        }

        // ========== UNWRAPPING MODE ==========
        if (isUnwrapping) {
            double unwrapError = unwrapTarget - currentTurretAngle;

            if (Math.abs(unwrapError) <= FIELD_TOLERANCE_DEG) {
                isUnwrapping = false;
                turret.stop();
                integralSum = 0;
                lastPower = 0.0;
                calculatedTurretTarget = unwrapTarget;
                return 0.0;
            }

            double power = calculatePID(unwrapError);
            if (INVERT_OUTPUT) power = -power;

            turret.setPower(power);
            lastPower = power;
            calculatedTurretTarget = unwrapTarget;

            sendGraphData(unwrapError, power, lastPTerm, lastITerm, lastDTerm);
            return power;
        }

        // ========== NORMAL FIELD TRACKING MODE ==========
        // Calculate current field facing
        double currentFieldFacing = normalizeAngle(robotHeadingDegrees + currentTurretAngle + TURRET_OFFSET);

        // Calculate field error
        double fieldError = normalizeAngle(TARGET_FIELD_ANGLE - currentFieldFacing);
        lastFieldError = fieldError;

        // Calculate optimal turret target (shortest path)
        double rawTurretTarget = TARGET_FIELD_ANGLE - robotHeadingDegrees - TURRET_OFFSET;
        double targetOption1 = rawTurretTarget;
        double targetOption2 = rawTurretTarget + 360.0;
        double targetOption3 = rawTurretTarget - 360.0;

        double diff1 = Math.abs(targetOption1 - currentTurretAngle);
        double diff2 = Math.abs(targetOption2 - currentTurretAngle);
        double diff3 = Math.abs(targetOption3 - currentTurretAngle);

        if (diff2 < diff1 && diff2 < diff3) {
            calculatedTurretTarget = targetOption2;
        } else if (diff3 < diff1 && diff3 < diff2) {
            calculatedTurretTarget = targetOption3;
        } else {
            calculatedTurretTarget = targetOption1;
        }

        // Recalculate field error based on optimized target
        double optimizedFieldTarget = normalizeAngle(robotHeadingDegrees + calculatedTurretTarget + TURRET_OFFSET);
        fieldError = optimizedFieldTarget - currentFieldFacing;
        lastFieldError = fieldError;

        // Check turret limits
        if (USE_LIMITS) {
            if (calculatedTurretTarget < TURRET_MIN_ANGLE) {
                double limitedFieldTarget = normalizeAngle(robotHeadingDegrees + TURRET_MIN_ANGLE + TURRET_OFFSET);
                fieldError = limitedFieldTarget - currentFieldFacing;
                calculatedTurretTarget = TURRET_MIN_ANGLE;
            } else if (calculatedTurretTarget > TURRET_MAX_ANGLE) {
                double limitedFieldTarget = normalizeAngle(robotHeadingDegrees + TURRET_MAX_ANGLE + TURRET_OFFSET);
                fieldError = limitedFieldTarget - currentFieldFacing;
                calculatedTurretTarget = TURRET_MAX_ANGLE;
            }
        }

        // ========== HYSTERESIS DEADBAND (anti-jitter!) ==========
        double absError = Math.abs(fieldError);

        if (isLocked) {
            // Currently locked - only unlock if error exceeds exit threshold
            if (absError > DEADBAND_EXIT) {
                isLocked = false;
                // Continue to PID control below
            } else {
                // Stay locked - don't move
                turret.stop();
                lastPower = 0.0;
                integralSum = 0;  // Reset integral while locked
                sendGraphData(fieldError, 0, 0, 0, 0);
                return 0.0;
            }
        } else {
            // Currently active - lock if error is small enough
            if (absError <= DEADBAND_ENTER) {
                isLocked = true;
                turret.stop();
                lastPower = 0.0;
                integralSum = 0;
                sendGraphData(fieldError, 0, 0, 0, 0);
                return 0.0;
            }
        }

        // ========== PID CONTROL ==========
        double power = calculatePID(fieldError);

        if (INVERT_OUTPUT) {
            power = -power;
        }

        turret.setPower(power);
        lastPower = power;

        sendGraphData(fieldError, power, lastPTerm, lastITerm, lastDTerm);
        return power;
    }

    /**
     * Calculate PID output based on field error
     * Uses different PID gains for clockwise vs counter-clockwise
     */
    private double calculatePID(double error) {
        long currentTime = System.currentTimeMillis();
        double dt = (lastUpdateTime == 0) ? 0.02 : (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        dt = Math.max(0.001, Math.min(0.1, dt));

        // Select PID gains based on error direction
        // Positive error = need to turn clockwise
        // Negative error = need to turn counter-clockwise
        double kP, kI, kD;
        if (error >= 0) {
            // Clockwise
            kP = CW_kP;
            kI = CW_kI;
            kD = CW_kD;
        } else {
            // Counter-clockwise
            kP = CCW_kP;
            kI = CCW_kI;
            kD = CCW_kD;
        }

        // P term
        lastPTerm = kP * error;

        // I term with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-FIELD_INTEGRAL_LIMIT, Math.min(FIELD_INTEGRAL_LIMIT, integralSum));
        lastITerm = kI * integralSum;

        // D term
        double derivative = (error - lastError) / dt;
        lastDTerm = kD * derivative;
        lastError = error;

        // Sum
        double output = lastPTerm + lastITerm + lastDTerm;

        // Apply limits
        double absOutput = Math.abs(output);
        if (absOutput > MAX_POWER) {
            output = Math.copySign(MAX_POWER, output);
        } else if (absOutput > 0.001 && absOutput < MIN_POWER) {
            output = Math.copySign(MIN_POWER, output);
        }

        return output;
    }

    /**
     * Send data to FTC Dashboard for graphing
     */
    private void sendGraphData(double error, double power, double pTerm, double iTerm, double dTerm) {
        if (!ENABLE_GRAPHING) return;

        TelemetryPacket packet = new TelemetryPacket();

        // These will show as separate lines on the graph
        packet.put("Turret/Error", error);
        packet.put("Turret/Power", power * 100);  // Scale to make visible with error
        packet.put("Turret/P_Term", pTerm * 100);
        packet.put("Turret/I_Term", iTerm * 100);
        packet.put("Turret/D_Term", dTerm * 100);
        packet.put("Turret/Locked", isLocked ? 1 : 0);
        packet.put("Turret/Direction", error >= 0 ? 1 : -1);  // 1=CW, -1=CCW

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Normalize angle to -180 to +180
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Check if turret is aligned with field target
     */
    public boolean isAligned() {
        return Math.abs(lastFieldError) <= FIELD_TOLERANCE_DEG;
    }

    /**
     * Get the calculated turret target angle
     */
    public double getCalculatedTurretTarget() {
        return calculatedTurretTarget;
    }

    /**
     * Get current turret angle (absolute)
     */
    public double getCurrentTurretAngle() {
        return turret.getTurretAngle();
    }

    /**
     * Get the field angle the turret is currently facing
     */
    public double getCurrentFieldFacing() {
        return normalizeAngle(lastRobotHeading + turret.getTurretAngle() + TURRET_OFFSET);
    }

    /**
     * Get error from target field angle
     */
    public double getFieldError() {
        return lastFieldError;
    }

    /**
     * Get last power output
     */
    public double getLastPower() {
        return lastPower;
    }

    /**
     * Get last robot heading
     */
    public double getLastRobotHeading() {
        return lastRobotHeading;
    }

    /**
     * Check if turret is currently unwrapping for wire safety
     */
    public boolean isUnwrapping() {
        return isUnwrapping;
    }

    /**
     * Get the unwrap target angle (only valid when isUnwrapping() is true)
     */
    public double getUnwrapTarget() {
        return unwrapTarget;
    }

    // ========== GETTERS FOR PID TERMS (for debugging) ==========
    public double getLastPTerm() { return lastPTerm; }
    public double getLastITerm() { return lastITerm; }
    public double getLastDTerm() { return lastDTerm; }
}