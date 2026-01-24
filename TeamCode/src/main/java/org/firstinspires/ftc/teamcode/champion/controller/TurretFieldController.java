package org.firstinspires.ftc.teamcode.champion.controller;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


/**
 * Field-Centric Turret Controller with Flyby Auto-Aim Strategy
 *
 * AUTO-AIM STRATEGY:
 * - CW turn: Constant power flyby → CW_CORRECTION PID (after passing target)
 * - CCW turn: Always CCW PID
 * - No deadband (direct continuous control)
 *
 * STATE MACHINE:
 * - CW_FLYBY: Constant power CW, waiting to pass target
 * - CW_CORRECTION: PID correction after CW flyby overshoot
 * - CCW_PID: Smooth PID for CCW movements
 *
 * DASHBOARD GRAPHING:
 * - Graphs error, power, state, P/I/D terms for tuning
 * - View in FTC Dashboard under "Turret" graph group
 */
@Config
public class TurretFieldController {


    // ========== AUTO-AIM STATE MACHINE ==========
    public enum AimState {
        CW_FLYBY,       // Constant velocity CW, waiting to overshoot
        CW_CORRECTION,  // PID correction after CW flyby
        CCW_PID         // PID for CCW movements
    }


    private final TurretController turret;
    private final FtcDashboard dashboard;


    // ========== FIELD TARGET ==========
    public static double TARGET_FIELD_ANGLE = 27.0;


    // ========== FLYBY PARAMETERS ==========
    public static double CW_FLYBY_POWER = 0.2;
    public static double FLYBY_OVERSHOOT_MARGIN = 3.0;
    public static double FLYBY_MIN_ERROR = 5.0;


    // ========== CW CORRECTION PID (after flyby overshoot) ==========
    public static double CW_CORR_kP = 0.05;
    public static double CW_CORR_kI = 0.0;
    public static double CW_CORR_kD = 0;


    // ========== CCW PID (for all CCW movements) ==========
    public static double CCW_kP = 0.06;
    public static double CCW_kI = 0.0;
    public static double CCW_kD = 0.005;


    public static double INTEGRAL_LIMIT = 20.0;


    // ========== POWER LIMITS ==========
    public static double MAX_POWER = 0.6;
    public static double MIN_POWER = 0.08;


    // ========== TOLERANCE (for isAligned check only) ==========
    public static double TOLERANCE_DEG = 1.5;


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
    private AimState aimState = AimState.CCW_PID;  // Default state
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
        aimState = AimState.CCW_PID;
        resetPID();
    }


    /**
     * Disable field-centric control
     */
    public void disable() {
        enabled = false;
        isUnwrapping = false;
        aimState = AimState.CCW_PID;
        turret.stop();
    }


    public boolean isEnabled() {
        return enabled;
    }


    /**
     * Get current aim state
     */
    public AimState getAimState() {
        return aimState;
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


            if (Math.abs(unwrapError) <= TOLERANCE_DEG) {
                isUnwrapping = false;
                turret.stop();
                integralSum = 0;
                lastPower = 0.0;
                calculatedTurretTarget = unwrapTarget;
                return 0.0;
            }


            double power = calculatePID(unwrapError, false);  // Use CCW PID for unwrap
            if (INVERT_OUTPUT) power = -power;


            turret.setPower(power);
            lastPower = power;
            calculatedTurretTarget = unwrapTarget;


            sendGraphData(unwrapError, power, lastPTerm, lastITerm, lastDTerm);
            return power;
        }


        // ========== CALCULATE FIELD ERROR ==========
        double currentFieldFacing = normalizeAngle(robotHeadingDegrees + currentTurretAngle + TURRET_OFFSET);
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


        // ========== AUTO-AIM STATE MACHINE (NO DEADBAND) ==========
        double power = 0.0;


        // Determine state transitions based on error
        if (fieldError > FLYBY_MIN_ERROR) {
            // Large CW error → enter flyby mode
            if (aimState != AimState.CW_FLYBY) {
                aimState = AimState.CW_FLYBY;
                resetPID();
            }
        } else if (fieldError < 0) {
            // Negative error (CCW needed) → CCW PID
            if (aimState == AimState.CW_FLYBY) {
                // Just finished flyby, switch to correction
                aimState = AimState.CW_CORRECTION;
                resetPID();
            } else if (aimState != AimState.CW_CORRECTION) {
                // Direct CCW movement
                aimState = AimState.CCW_PID;
            }
        } else if (fieldError >= 0 && fieldError <= FLYBY_MIN_ERROR) {
            // Small positive error
            if (aimState == AimState.CW_FLYBY && fieldError <= FLYBY_OVERSHOOT_MARGIN) {
                // Flyby passed target, switch to correction
                aimState = AimState.CW_CORRECTION;
                resetPID();
            } else if (aimState == AimState.CCW_PID) {
                // Was doing CCW, now small CW error - keep using CCW PID
                // (this handles oscillation around target smoothly)
            }
            // If already in CW_CORRECTION, stay there
        }


        // Execute current state
        switch (aimState) {
            case CW_FLYBY:
                // Constant power flyby
                power = CW_FLYBY_POWER;
                if (INVERT_OUTPUT) power = -power;


                turret.setPower(power);
                lastPower = power;
                lastPTerm = 0;
                lastITerm = 0;
                lastDTerm = 0;


                sendGraphData(fieldError, power, 0, 0, 0);
                return power;


            case CW_CORRECTION:
                // PID correction after flyby using CW_CORR gains
                power = calculatePID(fieldError, true);  // true = use CW correction gains
                if (INVERT_OUTPUT) power = -power;


                turret.setPower(power);
                lastPower = power;


                sendGraphData(fieldError, power, lastPTerm, lastITerm, lastDTerm);
                return power;


            case CCW_PID:
                // CCW PID for all CCW movements
                power = calculatePID(fieldError, false);  // false = use CCW gains
                if (INVERT_OUTPUT) power = -power;


                turret.setPower(power);
                lastPower = power;


                sendGraphData(fieldError, power, lastPTerm, lastITerm, lastDTerm);
                return power;
        }


        return 0.0;
    }


    /**
     * Calculate PID output
     *
     * @param error The field error in degrees
     * @param useCWCorrectionGains If true, use CW_CORR gains; if false, use CCW gains
     */
    private double calculatePID(double error, boolean useCWCorrectionGains) {
        long currentTime = System.currentTimeMillis();
        double dt = (lastUpdateTime == 0) ? 0.02 : (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;


        dt = Math.max(0.001, Math.min(0.1, dt));


        double kP, kI, kD;
        if (useCWCorrectionGains) {
            // CW correction gains (after flyby)
            kP = CW_CORR_kP;
            kI = CW_CORR_kI;
            kD = CW_CORR_kD;
        } else {
            // CCW gains (for all CCW movements)
            kP = CCW_kP;
            kI = CCW_kI;
            kD = CCW_kD;
        }


        // P term
        lastPTerm = kP * error;


        // I term with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
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


        packet.put("Turret/Error", error);
        packet.put("Turret/Power", power * 100);
        packet.put("Turret/P_Term", pTerm * 100);
        packet.put("Turret/I_Term", iTerm * 100);
        packet.put("Turret/D_Term", dTerm * 100);
        packet.put("Turret/State", aimState.ordinal());  // 0=CW_FLYBY, 1=CW_CORRECTION, 2=CCW_PID


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
        return Math.abs(lastFieldError) <= TOLERANCE_DEG;
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

