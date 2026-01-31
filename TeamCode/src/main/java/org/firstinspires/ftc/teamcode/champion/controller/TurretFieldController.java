package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Field-Centric Turret Controller with Dead Zone Handling
 *
 * Updated for Continuous Rotation Servo drive system.
 * Includes power ramping for smoother servo control.
 */
@Config
public class TurretFieldController {

    private final TurretController turret;
    private final FtcDashboard dashboard;

    // ========== FIELD TARGET ==========
    public static double TARGET_FIELD_ANGLE = 27.0;

    // ========== TURRET LIMITS (Dead Zone) ==========
    public static double CCW_LIMIT = 90.0;
    public static double CW_LIMIT = -90.0;
    public static boolean USE_LIMITS = true;

    // ========== CCW PID ==========
    // Tuned down for servos (they respond differently than motors)
    public static double CCW_kP = 0.025;
    public static double CCW_kI = 0.0;
    public static double CCW_kD = 0.002;

    // ========== CW PIDF ==========
    public static double CW_kP = 0.025;
    public static double CW_kI = 0.0;
    public static double CW_kD = 0.002;
    public static double CW_kF = 0.01;

    // ========== POWER LIMITS ==========
    // Servos often need lower max power for precision
    public static double MAX_POWER = 0.6;
    public static double MIN_POWER = 0.08;  // CR servos have larger deadbands
    public static double INTEGRAL_LIMIT = 15.0;

    // ========== SERVO RAMPING ==========
    // Prevents jerky motion from sudden power changes
    public static boolean USE_RAMPING = true;
    public static double RAMP_RATE = 2.0;  // Max power change per second

    // ========== ALIGNMENT ==========
    public static double FIELD_TOLERANCE_DEG = 2.0;  // Slightly larger for servo slop

    // ========== OUTPUT ==========
    public static boolean INVERT_OUTPUT = true;

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
    private double targetPower = 0.0;  // For ramping

    // PID state
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private long lastUpdateTime = 0;

    // For telemetry
    private double lastPTerm = 0.0;
    private double lastITerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastFTerm = 0.0;

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
        lastFTerm = 0.0;
        targetPower = 0.0;
        lastPower = 0.0;
    }

    /**
     * Main update - call every loop!
     * Call turret.update() BEFORE this!
     */
    public double update(double robotHeadingDegrees) {
        if (!enabled) {
            lastPower = 0.0;
            targetPower = 0.0;
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

        // ========== CALCULATE REQUIRED TURRET ANGLE ==========
        double requiredTurretAngle = TARGET_FIELD_ANGLE - robotHeadingDegrees - TURRET_OFFSET;
        requiredTurretAngle = normalizeAngle(requiredTurretAngle);
        debugRequiredTurret = requiredTurretAngle;

        // Calculate actual field facing (for debug)
        debugActualFieldFacing = normalizeAngle(robotHeadingDegrees + currentTurretAngle + TURRET_OFFSET);

        // ========== DEAD ZONE CHECK ==========
        if (USE_LIMITS) {
            if (requiredTurretAngle > CCW_LIMIT) {
                inDeadZone = true;
                requiredTurretAngle = CCW_LIMIT;
            } else if (requiredTurretAngle < CW_LIMIT) {
                inDeadZone = true;
                requiredTurretAngle = CW_LIMIT;
            } else {
                inDeadZone = false;
            }
        } else {
            inDeadZone = false;
        }

        // ========== CALCULATE ERROR ==========
        double error = requiredTurretAngle - currentTurretAngle;
        lastFieldError = error;

        // If in dead zone and at limit, stop
        if (inDeadZone) {
            boolean atCCWLimit = Math.abs(currentTurretAngle - CCW_LIMIT) < 3.0;
            boolean atCWLimit = Math.abs(currentTurretAngle - CW_LIMIT) < 3.0;

            if ((requiredTurretAngle >= CCW_LIMIT && atCCWLimit) ||
                    (requiredTurretAngle <= CW_LIMIT && atCWLimit)) {
                turret.stop();
                lastPower = 0.0;
                targetPower = 0.0;
                resetPID();
                sendGraphData(error, 0, 0, 0, 0, 0, true);
                return 0.0;
            }
        }

        // Calculate robot rotation rate for feedforward
        double robotRotationRate = normalizeAngle(robotHeadingDegrees - prevRobotHeading) / dt;

        // ========== DIRECTION-DEPENDENT CONTROL ==========
        double output;
        if (error >= 0) {
            output = calculateCW_PIDF(error, robotRotationRate, dt);
        } else {
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

        // ========== SERVO RAMPING ==========
        targetPower = output;
        if (USE_RAMPING) {
            output = applyRamping(lastPower, targetPower, dt);
        }

        turret.setPower(output);
        lastPower = output;

        sendGraphData(error, output, lastPTerm, lastITerm, lastDTerm, lastFTerm, inDeadZone);
        return output;
    }

    /**
     * Smooth power transitions for CR servos
     */
    private double applyRamping(double currentPower, double targetPower, double dt) {
        double maxChange = RAMP_RATE * dt;
        double delta = targetPower - currentPower;

        if (Math.abs(delta) <= maxChange) {
            return targetPower;
        }

        return currentPower + Math.copySign(maxChange, delta);
    }

    private double calculateCW_PIDF(double error, double robotRotationRate, double dt) {
        lastPTerm = CW_kP * error;

        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        lastITerm = CW_kI * integralSum;

        double errorDerivative = (error - lastError) / dt;
        lastDTerm = CW_kD * errorDerivative;
        lastError = error;

        lastFTerm = CW_kF * (-robotRotationRate);

        return lastPTerm + lastITerm + lastDTerm + lastFTerm;
    }

    private double calculateCCW_PID(double error, double dt) {
        lastPTerm = CCW_kP * error;

        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        lastITerm = CCW_kI * integralSum;

        double errorDerivative = (error - lastError) / dt;
        lastDTerm = CCW_kD * errorDerivative;
        lastError = error;

        lastFTerm = 0.0;

        return lastPTerm + lastITerm + lastDTerm;
    }

    private void sendGraphData(double error, double power, double pTerm, double iTerm, double dTerm, double fTerm, boolean deadZone) {
        if (!ENABLE_GRAPHING) return;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret/Error", error);
        packet.put("Turret/Power", power * 100);
        packet.put("Turret/TargetPower", targetPower * 100);
        packet.put("Turret/P", pTerm * 100);
        packet.put("Turret/I", iTerm * 100);
        packet.put("Turret/D", dTerm * 100);
        packet.put("Turret/F", fTerm * 100);
        packet.put("Turret/DeadZone", deadZone ? 1 : 0);
        packet.put("Turret/Dir", error >= 0 ? 1 : -1);

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
    public double getLastFTerm() { return lastFTerm; }

    // ========== AUTO-AIM ==========

    public interface HeadingProvider {
        double getHeadingDegrees();
    }

    public interface OpModeChecker {
        boolean isActive();
    }

    public static double AUTO_AIM_TIMEOUT_MS = 2500.0;  // Slightly longer for servos

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
        final int REQUIRED_ALIGNED_CYCLES = 5;  // More cycles for servo settling

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
                telemetry.addData("Robot Heading", "%.1f°", heading);
                telemetry.addData("Turret Angle", "%.1f°", turret.getTurretAngle());
                telemetry.addData("Required Turret", "%.1f°", debugRequiredTurret);
                telemetry.addData("Field Target", "%.1f°", TARGET_FIELD_ANGLE);
                telemetry.addData("Actual Facing", "%.1f°", debugActualFieldFacing);
                telemetry.addData("Error", "%.1f°", lastFieldError);
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
}