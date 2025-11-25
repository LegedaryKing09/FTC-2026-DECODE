package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controller for turret with encoder feedback and PID control
 */
@Config
public class TurretController {

    // PID Constants (tunable via FTC Dashboard)
    public static double TURRET_Kp = 0.03;
    public static double TURRET_Ki = 0.0;
    public static double TURRET_Kd = 0.002;

    // Control parameters (tunable via FTC Dashboard)
    public static double TURRET_MAX_POWER = 0.6;           // Maximum servo power
    public static double TURRET_POSITION_TOLERANCE = 2.0;  // Degrees - when to stop
    public static double TURRET_MIN_POWER = 0.08;          // Minimum power to overcome friction
    public static double TURRET_TIMEOUT_SECONDS = 3.0;     // Timeout for moves
    public static double TURRET_INIT_POSITION = 0.0;       // Initialization target position
    public static double TURRET_MANUAL_SPEED = 0.6;        // Manual control speed
    public static double TURRET_JOYSTICK_DEADBAND = 0.1;   // Joystick deadband

    // Limits (tunable via FTC Dashboard)
    public static double TURRET_MIN_ANGLE = 0.0;
    public static double TURRET_MAX_ANGLE = 144.0;
    public static boolean TURRET_ENABLE_LIMITS = false;

    // Presets (tunable via FTC Dashboard)
    public static double TURRET_FRONT_POSITION = 0.0;
    public static double TURRET_LEFT_POSITION = 36.0;
    public static double TURRET_BACK_POSITION = 72.0;
    public static double TURRET_RIGHT_POSITION = 108.0;

    // Constants
    private static final double VOLTAGE_TO_DEGREES = 360.0 / 3.3;
    private static final double GEAR_RATIO = 2.5;

    private CRServo turretServo;
    private AnalogInput turretEncoder;
    private ElapsedTime runtime;

    // Position tracking
    private double lastPosition = 0;
    private double currentPosition = 0;
    private double velocity = 0;
    private double lastVelocityTime = 0;

    // PID state
    private double targetPosition = 0;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private boolean pidActive = false;
    private double moveStartTime = 0;

    // Initialization state
    private boolean isInitialized = false;

    // Manual control override
    private boolean manualOverride = false;

    public TurretController(CRServo servo, AnalogInput encoder, ElapsedTime time) {
        this.turretServo = servo;
        this.turretEncoder = encoder;
        this.runtime = time;
        this.lastTime = time.seconds();
        this.lastVelocityTime = time.seconds();

        // Initialize position
        updatePosition();
        targetPosition = currentPosition;
    }

    /**
     * Initialize turret to configured position
     * Call this during initialization or at the start of autonomous/teleop
     */
    public void initialize() {
        targetPosition = TURRET_INIT_POSITION;
        pidActive = true;
        resetPID();
        moveStartTime = runtime.seconds();
        isInitialized = false;
        manualOverride = false;
    }

    /**
     * Check if initialization is complete
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Update position from encoder and handle PID control
     * MUST be called every loop!
     */
    public void update() {
        updatePosition();

        // Handle PID control
        double power = 0;

        if (!manualOverride && pidActive) {
            power = calculatePID();

            // Check if target reached
            double error = getShortestAngleError(targetPosition, currentPosition);

            if (Math.abs(error) < TURRET_POSITION_TOLERANCE) {
                pidActive = false;
                power = 0;
                integral = 0;

                // Mark as initialized if we were initializing
                if (!isInitialized && Math.abs(targetPosition - TURRET_INIT_POSITION) < TURRET_POSITION_TOLERANCE) {
                    isInitialized = true;
                }
            }

            // Check for timeout
            double currentTime = runtime.seconds();
            if (currentTime - moveStartTime > TURRET_TIMEOUT_SECONDS) {
                pidActive = false;
                power = 0;
                integral = 0;
            }

            setPower(power);
        }
    }

    /**
     * Update position from encoder
     */
    private void updatePosition() {
        if (turretEncoder == null) return;

        double voltage = turretEncoder.getVoltage();
        double servoPosition = voltage * VOLTAGE_TO_DEGREES;
        currentPosition = servoPosition / GEAR_RATIO;

        // Calculate velocity
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastVelocityTime;

        if (deltaTime > 0.1) {
            double deltaPosition = currentPosition - lastPosition;

            // Handle wraparound
            if (deltaPosition > 180) deltaPosition -= 360;
            else if (deltaPosition < -180) deltaPosition += 360;

            velocity = deltaPosition / deltaTime;
            lastPosition = currentPosition;
            lastVelocityTime = currentTime;
        }
    }

    /**
     * Calculate PID output
     */
    private double calculatePID() {
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;

        if (deltaTime <= 0) return 0;

        double error = getShortestAngleError(targetPosition, currentPosition);

        // Proportional term
        double P = TURRET_Kp * error;

        // Integral term (with anti-windup)
        integral += error * deltaTime;
        // Limit integral to prevent windup
        double maxIntegral = TURRET_MAX_POWER / (TURRET_Ki + 0.0001);
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double I = TURRET_Ki * integral;

        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        double D = TURRET_Kd * derivative;

        // Calculate total output
        double output = P + I + D;

        // Apply minimum power to overcome friction
        if (Math.abs(output) > 0.01 && Math.abs(output) < TURRET_MIN_POWER) {
            output = Math.signum(output) * TURRET_MIN_POWER;
        }

        // Clamp output to max power
        output = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, output));

        // Apply soft limits if enabled
        if (TURRET_ENABLE_LIMITS) {
            if (currentPosition <= TURRET_MIN_ANGLE && output < 0) output = 0;
            if (currentPosition >= TURRET_MAX_ANGLE && output > 0) output = 0;
        }

        lastError = error;
        lastTime = currentTime;

        return output;
    }

    /**
     * Calculate shortest angle error (handles wraparound)
     */
    private double getShortestAngleError(double target, double current) {
        double error = target - current;

        // Normalize to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    /**
     * Reset PID state
     */
    private void resetPID() {
        integral = 0;
        lastError = 0;
        lastTime = runtime.seconds();
    }

    /**
     * Set target position for automatic control
     */
    public void setTargetPosition(double target) {
        targetPosition = target;

        // Wrap target to valid range
        while (targetPosition >= 360) targetPosition -= 360;
        while (targetPosition < 0) targetPosition += 360;

        pidActive = true;
        manualOverride = false;
        resetPID();
        moveStartTime = runtime.seconds();
    }

    /**
     * Increment turret position by specified degrees
     */
    public void incrementPosition(double degrees) {
        targetPosition = currentPosition + degrees;

        // Wrap target to valid range
        while (targetPosition >= 360) targetPosition -= 360;
        while (targetPosition < 0) targetPosition += 360;

        pidActive = true;
        manualOverride = false;
        resetPID();
        moveStartTime = runtime.seconds();
    }

    /**
     * Decrement turret position by specified degrees
     */
    public void decrementPosition(double degrees) {
        incrementPosition(-degrees);
    }

    /**
     * Go to preset positions
     */
    public void goToFront() {
        setTargetPosition(TURRET_FRONT_POSITION);
    }

    public void goToLeft() {
        setTargetPosition(TURRET_LEFT_POSITION);
    }

    public void goToBack() {
        setTargetPosition(TURRET_BACK_POSITION);
    }

    public void goToRight() {
        setTargetPosition(TURRET_RIGHT_POSITION);
    }

    /**
     * Cancel automatic positioning
     */
    public void cancelPositionMode() {
        pidActive = false;
        manualOverride = false;
        integral = 0;
    }

    /**
     * Set the turret power directly (manual control)
     */
    public void setPower(double power) {
        if (turretServo != null) {
            // Apply soft limits if enabled
            if (TURRET_ENABLE_LIMITS) {
                if (currentPosition <= TURRET_MIN_ANGLE && power < 0) power = 0;
                if (currentPosition >= TURRET_MAX_ANGLE && power > 0) power = 0;
            }

            turretServo.setPower(power);

            // If manually controlling, disable PID
            if (Math.abs(power) > 0.01) {
                pidActive = false;
                manualOverride = true;
                integral = 0;
            } else {
                manualOverride = false;
            }
        }
    }

    /**
     * Handle joystick input for manual control
     */
    public void handleManualInput(double joystickInput) {
        if (Math.abs(joystickInput) > TURRET_JOYSTICK_DEADBAND) {
            setPower(joystickInput * TURRET_MANUAL_SPEED);
        } else if (manualOverride) {
            setPower(0);
            manualOverride = false;
        }
    }

    /**
     * Stop the turret
     */
    public void stop() {
        pidActive = false;
        manualOverride = false;
        integral = 0;
        if (turretServo != null) {
            turretServo.setPower(0);
        }
    }

    /**
     * Check if PID is actively controlling
     */
    public boolean isMoving() {
        return pidActive;
    }

    /**
     * Check if target has been reached
     */
    public boolean atTarget() {
        return !pidActive && Math.abs(getShortestAngleError(targetPosition, currentPosition)) < TURRET_POSITION_TOLERANCE;
    }

    /**
     * Check if near a target position
     */
    public boolean isNear(double target) {
        double error = Math.abs(getShortestAngleError(target, currentPosition));
        return error < TURRET_POSITION_TOLERANCE * 2;
    }

    // Getters
    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getPositionError() {
        return getShortestAngleError(targetPosition, currentPosition);
    }

    public double getVelocity() {
        return velocity;
    }

    public boolean isPositionMode() {
        return pidActive;
    }

    public boolean isManualOverride() {
        return manualOverride;
    }

    public double getRawVoltage() {
        return turretEncoder != null ? turretEncoder.getVoltage() : 0;
    }

    public double getServoPosition() {
        return getRawVoltage() * VOLTAGE_TO_DEGREES;
    }

    public double getCurrentPower() {
        return turretServo != null ? turretServo.getPower() : 0;
    }

    /**
     * Get PID components (for debugging)
     */
    public double getProportional() {
        return TURRET_Kp * getPositionError();
    }

    public double getIntegral() {
        return TURRET_Ki * integral;
    }

    public double getDerivative() {
        return TURRET_Kd * (getPositionError() - lastError);
    }
}