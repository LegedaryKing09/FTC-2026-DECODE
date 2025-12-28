package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;

/**
 * Field-Centric Turret Controller with PID
 *
 * Makes the turret always face a specific FIELD direction, regardless of robot rotation.
 * Has its own PID for smooth tracking as robot rotates.
 *
 * IMPORTANT: Turret angle is set to 0 ONCE at startup, then remains absolute.
 * Do NOT reset turret angle during operation!
 *
 * HOW IT WORKS:
 * - Robot starts facing forward = 0° field heading (IMU reset once)
 * - Turret starts centered = 0° turret angle (set ONCE at startup)
 * - TARGET_FIELD_ANGLE = direction of AprilTag in field coordinates
 * - As robot rotates, turret counter-rotates to maintain aim
 *
 * THE MATH:
 *   currentFieldFacing = robotHeading + turretAngle
 *   fieldError = targetFieldAngle - currentFieldFacing
 *   PID controls based on fieldError for smooth tracking
 */
@Config
public class TurretFieldController {

    private final TurretController turret;

    // ========== FIELD TARGET ==========
    public static double TARGET_FIELD_ANGLE = 45.0; //needs tuning with limelight correction

    // ========== FIELD PID (tune these for smooth tracking) ==========
    public static double FIELD_kP = 0.07;       // Proportional - response speed
    public static double FIELD_kI = 0.0;        // Integral - usually 0
    public static double FIELD_kD = 0.008;      // Derivative - smoothness/damping
    public static double FIELD_INTEGRAL_LIMIT = 20.0;

    public static double MAX_POWER = 0.7;
    public static double MIN_POWER = 0.10;

    public static double FIELD_TOLERANCE_DEG = 2.0;
    public static boolean INVERT_OUTPUT = true;

    // ========== TURRET LIMITS ==========
    public static double TURRET_MIN_ANGLE = -150.0;
    public static double TURRET_MAX_ANGLE = 150.0;
    public static boolean USE_LIMITS = true;

    // ========== WIRE SAFETY ==========
    // Absolute angle threshold to trigger unwrapping (prevents wire damage)
    public static double WIRE_SAFETY_THRESHOLD = 180.0;
    public static boolean USE_WIRE_SAFETY = true;

    // ========== OFFSET ==========
    // If turret 0° doesn't align with robot forward
    public static double TURRET_OFFSET = 0.0;

    // ========== PID STATE ==========
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastUpdateTime = 0;

    // ========== STATE ==========
    private boolean enabled = false;
    private double lastRobotHeading = 0.0;
    private double calculatedTurretTarget = 0.0;
    private double lastPower = 0.0;

    // Wire safety unwrapping state
    private boolean isUnwrapping = false;
    private double unwrapTarget = 0.0;

    public TurretFieldController(TurretController turret) {
        this.turret = turret;
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
        resetPID();
    }

    /**
     * Disable field-centric control
     */
    public void disable() {
        enabled = false;
        isUnwrapping = false;
        turret.stop();
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Reset PID state (does NOT reset turret angle!)
     */
    public void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
        lastUpdateTime = 0;
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
            return 0.0;
        }

        lastRobotHeading = robotHeadingDegrees;

        // Get current turret angle (absolute, never reset during operation)
        double currentTurretAngle = turret.getTurretAngle();

        // ========== WIRE SAFETY CHECK ==========
        // If turret has rotated beyond safe threshold, initiate unwrap
        if (USE_WIRE_SAFETY && !isUnwrapping) {
            if (Math.abs(currentTurretAngle) > WIRE_SAFETY_THRESHOLD) {
                // Turret is beyond safe zone - need to unwrap by going 360° opposite direction
                // This brings us back to same field position but with safer turret angle
                isUnwrapping = true;
                if (currentTurretAngle > WIRE_SAFETY_THRESHOLD) {
                    // Rotated too far positive, unwrap by going -360°
                    unwrapTarget = currentTurretAngle - 360.0;
                } else {
                    // Rotated too far negative, unwrap by going +360°
                    unwrapTarget = currentTurretAngle + 360.0;
                }
            }
        }

        // ========== UNWRAPPING MODE ==========
        // If we're unwrapping, move to unwrap target instead of field target
        if (isUnwrapping) {
            double unwrapError = unwrapTarget - currentTurretAngle;

            // Check if unwrap complete
            if (Math.abs(unwrapError) <= FIELD_TOLERANCE_DEG) {
                isUnwrapping = false;
                turret.stop();
                integralSum = 0;
                lastPower = 0.0;
                calculatedTurretTarget = unwrapTarget;
                return 0.0;
            }

            // Calculate PID based on unwrap error
            double power = calculatePID(unwrapError);

            // Apply direction inversion
            if (INVERT_OUTPUT) {
                power = -power;
            }

            // Apply power
            turret.setPower(power);
            lastPower = power;
            calculatedTurretTarget = unwrapTarget;

            return power;
        }

        // ========== NORMAL FIELD TRACKING MODE ==========
        // Calculate current field facing
        double currentFieldFacing = normalizeAngle(robotHeadingDegrees + currentTurretAngle + TURRET_OFFSET);

        // Calculate field error (what we need to correct)
        double fieldError = normalizeAngle(TARGET_FIELD_ANGLE - currentFieldFacing);

        // Calculate what turret angle we would need (unnormalized first)
        double rawTurretTarget = TARGET_FIELD_ANGLE - robotHeadingDegrees - TURRET_OFFSET;

        // Wire-safety optimization: choose shortest rotational path
        // If the difference between target and current is > 180°, go the other way
        double targetOption1 = rawTurretTarget;
        double targetOption2 = rawTurretTarget + 360.0;
        double targetOption3 = rawTurretTarget - 360.0;

        // Find which option requires the smallest rotation from current position
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

        // Recalculate field error based on the optimized turret target
        // This ensures we move in the shortest direction
        double optimizedFieldTarget = normalizeAngle(robotHeadingDegrees + calculatedTurretTarget + TURRET_OFFSET);
        fieldError = optimizedFieldTarget - currentFieldFacing;

        // Check turret limits - if target is outside limits, clamp the error
        if (USE_LIMITS) {
            if (calculatedTurretTarget < TURRET_MIN_ANGLE) {
                // Target is beyond min limit
                double limitedFieldTarget = normalizeAngle(robotHeadingDegrees + TURRET_MIN_ANGLE + TURRET_OFFSET);
                fieldError = limitedFieldTarget - currentFieldFacing;
                calculatedTurretTarget = TURRET_MIN_ANGLE;
            } else if (calculatedTurretTarget > TURRET_MAX_ANGLE) {
                // Target is beyond max limit
                double limitedFieldTarget = normalizeAngle(robotHeadingDegrees + TURRET_MAX_ANGLE + TURRET_OFFSET);
                fieldError = limitedFieldTarget - currentFieldFacing;
                calculatedTurretTarget = TURRET_MAX_ANGLE;
            }
        }

        // Check if aligned
        if (Math.abs(fieldError) <= FIELD_TOLERANCE_DEG) {
            turret.stop();
            integralSum = 0;
            lastPower = 0.0;
            return 0.0;
        }

        // Calculate PID based on field error
        double power = calculatePID(fieldError);

        // Apply direction inversion
        if (INVERT_OUTPUT) {
            power = -power;
        }

        // Apply power
        turret.setPower(power);
        lastPower = power;

        return power;
    }

    /**
     * Calculate PID output based on field error
     */
    private double calculatePID(double error) {
        long currentTime = System.currentTimeMillis();
        double dt = (lastUpdateTime == 0) ? 0.02 : (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        dt = Math.max(0.001, Math.min(0.1, dt));

        // P term
        double pTerm = FIELD_kP * error;

        // I term with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-FIELD_INTEGRAL_LIMIT, Math.min(FIELD_INTEGRAL_LIMIT, integralSum));
        double iTerm = FIELD_kI * integralSum;

        // D term - this helps smooth out rapid changes when robot rotates
        double derivative = (error - lastError) / dt;
        double dTerm = FIELD_kD * derivative;
        lastError = error;

        // Sum
        double output = pTerm + iTerm + dTerm;

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
        double fieldError = getFieldError();
        return Math.abs(fieldError) <= FIELD_TOLERANCE_DEG;
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
        return normalizeAngle(TARGET_FIELD_ANGLE - getCurrentFieldFacing());
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
}