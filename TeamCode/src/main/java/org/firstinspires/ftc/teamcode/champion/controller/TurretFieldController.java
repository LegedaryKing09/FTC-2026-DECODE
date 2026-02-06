package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Field-Centric Turret Controller with Dead Zone Handling
 * Updated for Continuous Rotation Servo drive system.
 *
 * DIRECTION CONVENTIONS:
 * - Turret: CW = positive, CCW = negative
 * - Robot Heading: CCW = positive, CW = negative (standard FTC)
 * - INVERT_HEADING = true compensates for this mismatch
 *
 * LIMITS:
 * - CW limit: +90°
 * - CCW limit: -179°
 *
 * FIXES:
 * - Added INVERT_HEADING to handle robot/turret direction mismatch
 * - Added error normalization to handle accumulated angle wraparound
 * - Fixed derivative kick by normalizing error change instead of error
 * - Fixed integral windup in dead zone
 * - Changed MIN_POWER to deadband instead of amplification
 */
@Config
public class TurretFieldController {

    private final TurretController turret;
    private final FtcDashboard dashboard;

    // ========== FIELD TARGET ==========
    public static double TARGET_FIELD_ANGLE = -45.0;

    // ========== TURRET LIMITS (Dead Zone) ==========
    // CW is positive, CCW is negative
    public static double CW_LIMIT = 120.0;     // Max clockwise
    public static double CCW_LIMIT = -179.0;  // Max counter-clockwise
    public static boolean USE_LIMITS = true;

    // ========== PID (unified for both directions) ==========
    public static double kP = 0.01;
    public static double kI = 0.004;
    public static double kD = 0.0005;

    // ========== POWER LIMITS ==========
    public static double MAX_POWER = 0.5;
    public static double MIN_POWER = 0.04;  // CR servos have larger deadbands - now used as deadband threshold
    public static double INTEGRAL_LIMIT = 15.0;

    // ========== ALIGNMENT ==========
    public static double FIELD_TOLERANCE_DEG = 2.0;

    // ========== OUTPUT ==========
    public static boolean INVERT_OUTPUT = false;

    // ========== HEADING DIRECTION ==========
    // Set to true if robot heading is CCW positive but turret is CW positive
    public static boolean INVERT_HEADING = true;

    // ========== OFFSET ==========
    public static double TURRET_OFFSET = 0.0;

    // ========== DASHBOARD ==========
    public static boolean ENABLE_GRAPHING = true;

    // ========== STATE ==========
    private boolean enabled = false;
    private boolean inDeadZone = false;
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

    // Debug
    private double debugRequiredTurret = 0.0;
    private double debugActualFieldFacing = 0.0;

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
        lastPower = 0.0;
    }

    /**
     * Main update - call every loop!
     * Call turret.update() BEFORE this!
     */
    public double update(double robotHeadingDegrees) {
        if (!enabled) {
            lastPower = 0.0;
            sendGraphData(0, 0, 0, 0, 0, false);
            return 0.0;
        }

        // Time delta
        long currentTime = System.currentTimeMillis();
        double dt = (lastUpdateTime == 0) ? 0.02 : (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        // FIX: Don't clamp max dt - let it reflect actual timing
        dt = Math.max(0.005, dt);
        // If loop is too slow, reset PID to avoid integral issues
        if (dt > 0.2) {
            resetPID();
            return 0.0;
        }

        lastRobotHeading = robotHeadingDegrees;

        // Apply heading inversion if robot heading convention is opposite to turret
        double effectiveHeading = INVERT_HEADING ? -robotHeadingDegrees : robotHeadingDegrees;

        double currentTurretAngle = turret.getTurretAngle();

        // ========== CALCULATE REQUIRED TURRET ANGLE ==========
        double requiredTurretAngle = TARGET_FIELD_ANGLE - effectiveHeading - TURRET_OFFSET;
        requiredTurretAngle = normalizeAngle(requiredTurretAngle);
        debugRequiredTurret = requiredTurretAngle;

        // Calculate actual field facing (for debug)
        debugActualFieldFacing = normalizeAngle(effectiveHeading + currentTurretAngle + TURRET_OFFSET);

        // ========== DEAD ZONE CHECK ==========
        // CW is positive (+90 limit), CCW is negative (-179 limit)
        if (USE_LIMITS) {
            if (requiredTurretAngle > CW_LIMIT) {
                inDeadZone = true;
                requiredTurretAngle = CW_LIMIT;
                integralSum = 0.0;  // FIX: Prevent integral windup
            } else if (requiredTurretAngle < CCW_LIMIT) {
                inDeadZone = true;
                requiredTurretAngle = CCW_LIMIT;
                integralSum = 0.0;  // FIX: Prevent integral windup
            } else {
                inDeadZone = false;
            }
        } else {
            inDeadZone = false;
        }

        // ========== CALCULATE ERROR ==========
        double error = requiredTurretAngle - currentTurretAngle;
        // Normalize error to handle accumulated angle wraparound
        error = normalizeAngle(error);
        lastFieldError = error;

        // If in dead zone and at limit, stop
        if (inDeadZone) {
            boolean atCWLimit = Math.abs(currentTurretAngle - CW_LIMIT) < 3.0;
            boolean atCCWLimit = Math.abs(currentTurretAngle - CCW_LIMIT) < 3.0;

            if ((requiredTurretAngle >= CW_LIMIT && atCWLimit) ||
                    (requiredTurretAngle <= CCW_LIMIT && atCCWLimit)) {
                turret.stop();
                lastPower = 0.0;
                resetPID();
                sendGraphData(error, 0, 0, 0, 0, true);
                return 0.0;
            }
        }

        // ========== PID CONTROL ==========
        double output = calculatePID(error, dt);

        // FIX: Use MIN_POWER as deadband threshold instead of amplification
        double absOutput = Math.abs(output);
        if (absOutput < MIN_POWER) {
            output = 0.0;  // Deadband - don't amplify weak signals
        } else if (absOutput > MAX_POWER) {
            output = Math.copySign(MAX_POWER, output);
        }

        // Apply inversion if needed
        if (INVERT_OUTPUT) {
            output = -output;
        }

        turret.setPower(output);
        lastPower = output;

        sendGraphData(error, output, lastPTerm, lastITerm, lastDTerm, inDeadZone);
        return output;
    }

    private double calculatePID(double error, double dt) {
        // P term
        lastPTerm = kP * error;

        // I term
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        lastITerm = kI * integralSum;

        // D term
        // FIX: Normalize the error change to prevent derivative kick from wraparound
        double errorChange = error - lastError;
        errorChange = normalizeAngle(errorChange);
        double errorDerivative = errorChange / dt;
        lastDTerm = kD * errorDerivative;
        lastError = error;

        return lastPTerm + lastITerm + lastDTerm;
    }

    private void sendGraphData(double error, double power, double pTerm, double iTerm, double dTerm, boolean deadZone) {
        if (!ENABLE_GRAPHING) return;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret/Error", error);
        packet.put("Turret/Power", power * 100);
        packet.put("Turret/P", pTerm * 100);
        packet.put("Turret/I", iTerm * 100);
        packet.put("Turret/D", dTerm * 100);
        packet.put("Turret/DeadZone", deadZone ? 1 : 0);

        packet.put("Turret/RobotHeading", lastRobotHeading);
        packet.put("Turret/TurretAngle", turret.getTurretAngle());
        packet.put("Turret/RequiredTurret", debugRequiredTurret);
        packet.put("Turret/ActualFieldFacing", debugActualFieldFacing);
        packet.put("Turret/FieldTarget", TARGET_FIELD_ANGLE);

        dashboard.sendTelemetryPacket(packet);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public boolean isAligned() {
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

    public double getRequiredTurretAngle() {
        return debugRequiredTurret;
    }

    public double getActualFieldFacing() {
        return debugActualFieldFacing;
    }

    public double getLastPTerm() { return lastPTerm; }
    public double getLastITerm() { return lastITerm; }
    public double getLastDTerm() { return lastDTerm; }
    public double getLastFTerm() { return 0.0; }  // Kept for compatibility

    // ========== AUTO-AIM ==========

    public interface HeadingProvider {
        double getHeadingDegrees();
    }

    public interface OpModeChecker {
        boolean isActive();
    }

    public static double AUTO_AIM_TIMEOUT_MS = 1800.0;
    public static double WRAP_POWER = 0.4;           // Power to use during wrap-around
    public static double WRAP_TIMEOUT_MS = 2500.0;   // Timeout for wrap phase
    public static double WRAP_MARGIN_DEG = 10.0;     // How far past the opposite limit to target during wrap

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
        final int REQUIRED_ALIGNED_CYCLES = 5;

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

            turret.update();
            double heading = headingProvider.getHeadingDegrees();
            update(heading);

            if (telemetry != null) {
                telemetry.addData("AutoAim", "Aiming...");
                telemetry.addData("Robot Heading", "%.1f\u00B0", heading);
                telemetry.addData("Turret Angle", "%.1f\u00B0", turret.getTurretAngle());
                telemetry.addData("Required Turret", "%.1f\u00B0", debugRequiredTurret);
                telemetry.addData("Field Target", "%.1f\u00B0", TARGET_FIELD_ANGLE);
                telemetry.addData("Actual Facing", "%.1f\u00B0", debugActualFieldFacing);
                telemetry.addData("Error", "%.1f\u00B0", lastFieldError);
                telemetry.addData("Dead Zone", inDeadZone);
                telemetry.addData("Power", "%.2f", lastPower);
                telemetry.update();
            }

            if (inDeadZone && Math.abs(lastPower) < 0.01) {
                if (telemetry != null) {
                    telemetry.addData("AutoAim", "IN DEAD ZONE");
                    telemetry.update();
                }
                return false;
            }

            if (isAligned()) {
                alignedCount++;
                if (alignedCount >= REQUIRED_ALIGNED_CYCLES) {
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

    // ========== AUTO-AIM WITH DEAD ZONE WRAP-AROUND (for Auton only) ==========

    /**
     * Auton-only auto-aim that wraps around the dead zone if the direct path is blocked.
     *
     * If the required turret angle falls in the dead zone (between CW_LIMIT and CCW_LIMIT
     * going through the back), this method will command the turret to spin the OTHER direction
     * all the way around to reach the target from the opposite side.
     *
     * Example: target requires +150 deg but CW_LIMIT is +120 deg.
     *   -> Instead of giving up, spin CCW through -179 and wrap to reach the equivalent angle.
     *
     * @param targetAngle     Desired field-relative angle
     * @param headingProvider  Provides current robot heading
     * @param opModeChecker    Checks if opmode is still active
     * @param telemetry        Optional telemetry for debug
     * @return true if aligned, false on timeout/failure
     */
    public boolean autoAimWithWrap(double targetAngle,
                                   HeadingProvider headingProvider,
                                   OpModeChecker opModeChecker,
                                   org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {

        setTargetFieldAngle(targetAngle);
        enable();

        long startTime = System.currentTimeMillis();
        int alignedCount = 0;
        final int REQUIRED_ALIGNED_CYCLES = 5;

        while (opModeChecker.isActive()) {
            long elapsed = System.currentTimeMillis() - startTime;

            if (elapsed > AUTO_AIM_TIMEOUT_MS + WRAP_TIMEOUT_MS) {
                if (telemetry != null) {
                    telemetry.addData("AutoAimWrap", "TIMEOUT");
                    telemetry.update();
                }
                disable();
                return false;
            }

            turret.update();
            double heading = headingProvider.getHeadingDegrees();
            update(heading);

            if (telemetry != null) {
                telemetry.addData("AutoAimWrap", "Aiming...");
                telemetry.addData("Turret Angle", "%.1f\u00B0", turret.getTurretAngle());
                telemetry.addData("Required Turret", "%.1f\u00B0", debugRequiredTurret);
                telemetry.addData("Error", "%.1f\u00B0", lastFieldError);
                telemetry.addData("Dead Zone", inDeadZone);
                telemetry.addData("Power", "%.2f", lastPower);
                telemetry.update();
            }

            // If we hit the dead zone, wrap around instead of giving up
            if (inDeadZone && Math.abs(lastPower) < 0.01) {
                if (telemetry != null) {
                    telemetry.addData("AutoAimWrap", "WRAPPING AROUND...");
                    telemetry.update();
                }

                boolean wrapSuccess = executeWrapAround(headingProvider, opModeChecker, telemetry);
                if (!wrapSuccess) {
                    disable();
                    return false;
                }
                // After wrap, reset PID and continue normal aiming loop
                resetPID();
                startTime = System.currentTimeMillis(); // Reset timeout for final alignment
                continue;
            }

            if (isAligned()) {
                alignedCount++;
                if (alignedCount >= REQUIRED_ALIGNED_CYCLES) {
                    if (telemetry != null) {
                        telemetry.addData("AutoAimWrap", "ALIGNED!");
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

    /** Convenience overload without telemetry */
    public boolean autoAimWithWrap(double targetAngle,
                                   HeadingProvider headingProvider,
                                   OpModeChecker opModeChecker) {
        return autoAimWithWrap(targetAngle, headingProvider, opModeChecker, null);
    }

    /**
     * Executes the wrap-around maneuver.
     *
     * Determines which limit the turret is stuck at, then drives it in the
     * OPPOSITE direction (temporarily ignoring limits) until it passes through
     * the other limit, at which point normal PID control can take over.
     *
     * If stuck at CW_LIMIT (+120):  drive CCW past CCW_LIMIT (-179) to wrap
     * If stuck at CCW_LIMIT (-179):  drive CW past CW_LIMIT (+120) to wrap
     */
    private boolean executeWrapAround(HeadingProvider headingProvider,
                                      OpModeChecker opModeChecker,
                                      org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {

        double currentAngle = turret.getTurretAngle();

        // Determine which limit we're stuck at and choose wrap direction
        boolean stuckAtCW = Math.abs(currentAngle - CW_LIMIT) < 5.0;
        boolean stuckAtCCW = Math.abs(currentAngle - CCW_LIMIT) < 5.0;

        if (!stuckAtCW && !stuckAtCCW) {
            // Not actually stuck at a limit, shouldn't happen but bail safely
            return false;
        }

        // If stuck at CW limit, wrap CCW (negative power) to reach past CCW limit
        // If stuck at CCW limit, wrap CW (positive power) to reach past CW limit
        double wrapPower = stuckAtCW ? -WRAP_POWER : WRAP_POWER;
        double targetLimitToPass = stuckAtCW ? (CCW_LIMIT + WRAP_MARGIN_DEG) : (CW_LIMIT - WRAP_MARGIN_DEG);

        // Temporarily disable limits so the turret can spin freely
        boolean originalUseLimits = USE_LIMITS;
        USE_LIMITS = false;

        long wrapStart = System.currentTimeMillis();

        while (opModeChecker.isActive()) {
            long elapsed = System.currentTimeMillis() - wrapStart;
            if (elapsed > WRAP_TIMEOUT_MS) {
                USE_LIMITS = originalUseLimits;
                turret.stop();
                if (telemetry != null) {
                    telemetry.addData("AutoAimWrap", "WRAP TIMEOUT");
                    telemetry.update();
                }
                return false;
            }

            turret.update();
            turret.setPower(wrapPower);
            currentAngle = turret.getTurretAngle();

            if (telemetry != null) {
                telemetry.addData("AutoAimWrap", "Wrapping %s", stuckAtCW ? "CCW" : "CW");
                telemetry.addData("Turret Angle", "%.1f\u00B0", currentAngle);
                telemetry.addData("Target to pass", "%.1f\u00B0", targetLimitToPass);
                telemetry.addData("Wrap Power", "%.2f", wrapPower);
                telemetry.update();
            }

            // Check if we've wrapped past the opposite limit
            if (stuckAtCW && currentAngle <= targetLimitToPass) {
                break; // Wrapped CCW past CCW limit zone
            }
            if (stuckAtCCW && currentAngle >= targetLimitToPass) {
                break; // Wrapped CW past CW limit zone
            }

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                USE_LIMITS = originalUseLimits;
                turret.stop();
                return false;
            }
        }

        // Re-enable limits and let normal PID take over
        USE_LIMITS = originalUseLimits;
        turret.stop();
        inDeadZone = false;
        return true;
    }
}