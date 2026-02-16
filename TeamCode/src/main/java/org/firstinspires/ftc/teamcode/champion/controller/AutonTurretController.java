package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class AutonTurretController {

    // ========== HARDWARE NAMES ==========
    public static String SERVO_NAME = "turret";
    public static String ANALOG_NAME = "turret_analog";

    // ========== ANALOG CALIBRATION ==========
    public static double VOLTAGE_MAX = 3.3;
    public static double VOLTAGE_TO_DEGREES = 360.0 / VOLTAGE_MAX;  // Scale factor
    public static double CENTER_ANGLE = 180.0;  // What angle is "forward" on your turret

    // ========== DIRECTION-DEPENDENT PID ==========
    // Clockwise (positive error → positive power)
    public static double CW_kP = 0.05;
    public static double CW_kI = 0.0;
    public static double CW_kD = 0.0;

    // Counter-clockwise (negative error → negative power)
    public static double CCW_kP = 0.06;
    public static double CCW_kI = 0.0;
    public static double CCW_kD = 0.005;

    public static double INTEGRAL_LIMIT = 20.0;

    // ========== POWER LIMITS ==========
    public static double MAX_POWER = 0.6;
    public static double MIN_POWER = 0.08;

    // ========== TOLERANCE ==========
    public static double ANGLE_TOLERANCE = 1.5;  // degrees
    public static double SETTLED_TIME_MS = 200;  // must stay in tolerance for this long

    // ========== TIMEOUT ==========
    public static double DEFAULT_TIMEOUT_MS = 3000;  // 3 seconds default timeout

    // ========== OUTPUT INVERSION ==========
    public static boolean INVERT_OUTPUT = true;

    // Hardware
    private final LinearOpMode opMode;
    private final CRServo servo;
    private final AnalogInput analog;

    // State
    private double targetAngle = CENTER_ANGLE;
    private boolean isActive = false;

    // PID state
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastUpdateTime = 0;

    // Settling detection
    private long settledStartTime = 0;
    private boolean hasSettled = false;

    /**
     * Constructor
     */
    public AutonTurretController(LinearOpMode opMode) {
        this.opMode = opMode;
        this.servo = opMode.hardwareMap.get(CRServo.class, SERVO_NAME);
        this.analog = opMode.hardwareMap.get(AnalogInput.class, ANALOG_NAME);
        this.targetAngle = getRawAngle();  // Start at current position
    }

    /**
     * Get raw angle from analog sensor (absolute position)
     * This directly reads the turret's actual angle
     */
    public double getRawAngle() {
        double voltage = analog.getVoltage();
        return voltage * VOLTAGE_TO_DEGREES;
    }

    /**
     * Get current turret angle (same as getRawAngle, for consistency)
     */
    public double getCurrentAngle() {
        return getRawAngle();
    }

    /**
     * Get raw analog voltage (for debugging/calibration)
     */
    public double getVoltage() {
        return analog.getVoltage();
    }

    /**
     * Set target angle and WAIT until reached (blocking)
     *
     * @param angleDegrees Target angle in absolute degrees (0-360)
     * @return true if target reached, false if timeout or opmode stopped
     */
    public boolean setAngleAndWait(double angleDegrees) {
        return setAngleAndWait(angleDegrees, DEFAULT_TIMEOUT_MS);
    }

    /**
     * Set target angle and WAIT until reached (blocking) with custom timeout
     *
     * @param angleDegrees Target angle in absolute degrees
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return true if target reached, false if timeout or opmode stopped
     */
    public boolean setAngleAndWait(double angleDegrees, double timeoutMs) {
        setTargetAngle(angleDegrees);

        long startTime = System.currentTimeMillis();

        while (opMode.opModeIsActive()) {
            update();

            // Check if reached target
            if (isAtTarget()) {
                stop();
                return true;
            }

            // Check timeout
            if (System.currentTimeMillis() - startTime > timeoutMs) {
                stop();
                return false;
            }

            opMode.sleep(20);
        }

        stop();
        return false;
    }

    /**
     * Set target angle (non-blocking)
     * Call update() in your loop to move towards target
     *
     * @param angleDegrees Target angle in absolute degrees
     */
    public void setTargetAngle(double angleDegrees) {
        // Normalize to 0-360
        targetAngle = normalizeAngle(angleDegrees);
        resetPID();
        isActive = true;
        hasSettled = false;
        settledStartTime = 0;
    }

    /**
     * Update the controller - call this in your loop!
     *
     * @return Current power being applied
     */
    public double update() {
        if (!isActive) {
            return 0.0;
        }

        double currentAngle = getRawAngle();

        // Calculate shortest path error (handles wraparound)
        double error = calculateShortestError(targetAngle, currentAngle);

        // Check if within tolerance
        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            if (settledStartTime == 0) {
                settledStartTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - settledStartTime >= SETTLED_TIME_MS) {
                hasSettled = true;
                stop();
                return 0.0;
            }
        } else {
            settledStartTime = 0;
            hasSettled = false;
        }

        // Calculate PID (using direction-dependent gains from your code)
        double power = calculatePID(error);

        if (INVERT_OUTPUT) {
            power = -power;
        }

        servo.setPower(power);
        return power;
    }

    /**
     * Calculate shortest angular error with wraparound handling
     * Returns positive for clockwise, negative for counter-clockwise
     */
    private double calculateShortestError(double target, double current) {
        double error = target - current;

        // Normalize to -180 to +180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    /**
     * Calculate PID output using direction-dependent gains
     */
    private double calculatePID(double error) {
        long currentTime = System.currentTimeMillis();
        double dt = (lastUpdateTime == 0) ? 0.02 : (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        // Clamp dt to reasonable values
        dt = Math.max(0.001, Math.min(0.1, dt));

        // Select PID gains based on error direction (from your code)
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
        double pTerm = kP * error;

        // I term with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        double iTerm = kI * integralSum;

        // D term
        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;
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
     * Normalize angle to 0-360 range
     */
    private double normalizeAngle(double angle) {
        while (angle >= 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }

    /**
     * Check if turret has reached and settled at target
     */
    public boolean isAtTarget() {
        return hasSettled;
    }

    /**
     * Get current error from target
     */
    public double getError() {
        return calculateShortestError(targetAngle, getRawAngle());
    }

    /**
     * Get target angle
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Stop the turret and deactivate controller
     */
    public void stop() {
        isActive = false;
        servo.setPower(0);
    }

    /**
     * Reset PID state
     */
    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
        lastUpdateTime = 0;
        settledStartTime = 0;
        hasSettled = false;
    }

    /**
     * Check if controller is currently active
     */
    public boolean isActive() {
        return isActive;
    }
}