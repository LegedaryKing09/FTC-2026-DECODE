package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;

/**
 * Field-Centric Turret Controller - Thin wrapper around TurretController.
 *
 * Now uses position servos (not CR servos), so no PID needed.
 * update(heading) just calls turret.updateAutoAim(heading).
 *
 * Keeps the same public API so AutonMethods doesn't need changes.
 */
@Config
public class TurretFieldController {

    private final TurretController turret;

    // ========== FIELD TARGET ==========
    public static double TARGET_FIELD_ANGLE = -45.0;

    // ========== STATE ==========
    private boolean enabled = false;

    // ========== INTERFACES (kept for AutonMethods compatibility) ==========
    public interface HeadingProvider {
        double getHeadingDegrees();
    }

    public interface OpModeChecker {
        boolean isActive();
    }

    public static double AUTO_AIM_TIMEOUT_MS = 1800.0;

    public TurretFieldController(TurretController turret) {
        this.turret = turret;
    }

    public void setTargetFieldAngle(double fieldAngleDegrees) {
        TARGET_FIELD_ANGLE = fieldAngleDegrees;
        turret.setFieldAngle(fieldAngleDegrees);
    }

    public double getTargetFieldAngle() {
        return TARGET_FIELD_ANGLE;
    }

    public void enable() {
        enabled = true;
        turret.setFieldAngle(TARGET_FIELD_ANGLE);
        turret.enableAutoAim();
    }

    public void disable() {
        enabled = false;
        turret.disableAutoAim();
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Main update - call every loop.
     * Just delegates to turret.updateAutoAim(heading).
     */
    public double update(double robotHeadingDegrees) {
        if (!enabled) return 0.0;

        turret.updateAutoAim(robotHeadingDegrees);
        return 0.0;
    }

    /**
     * Auto-aim with wrap — sets target and enables, then blocks until aligned or timeout.
     * For auton use.
     */
    public boolean autoAimWithWrap(double targetAngle,
                                   HeadingProvider headingProvider,
                                   OpModeChecker opModeChecker,
                                   org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        setTargetFieldAngle(targetAngle);
        enable();

        // Do one update to snap turret to position
        double heading = headingProvider.getHeadingDegrees();
        turret.updateAutoAim(heading);

        return true;
    }

    public boolean autoAimWithWrap(double targetAngle,
                                   HeadingProvider headingProvider,
                                   OpModeChecker opModeChecker) {
        return autoAimWithWrap(targetAngle, headingProvider, opModeChecker, null);
    }

    public boolean autoAim(HeadingProvider headingProvider, OpModeChecker opModeChecker) {
        return autoAimWithWrap(TARGET_FIELD_ANGLE, headingProvider, opModeChecker, null);
    }

    public boolean autoAim(double targetAngle, HeadingProvider headingProvider, OpModeChecker opModeChecker) {
        return autoAimWithWrap(targetAngle, headingProvider, opModeChecker, null);
    }

    // ========== COMPATIBILITY GETTERS ==========

    public boolean isAligned() { return true; }
    public boolean isInDeadZone() { return false; }
    public double getFieldError() { return 0.0; }
    public double getLastPower() { return 0.0; }
    public double getCurrentTurretAngle() { return turret.getTurretAngle(); }
    public void resetPID() {}
    public void applyLimelightCorrection(double tx) { turret.addAimOffset(tx); }
    public double getTurretOffset() { return turret.getAimOffset(); }
    public void resetTurretOffset() { turret.setAimOffset(0.0); }
}