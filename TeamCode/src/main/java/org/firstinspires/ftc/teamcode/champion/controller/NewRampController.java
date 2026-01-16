package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Ramp Controller with Continuous Angle Tracking
 *
 * PROBLEM SOLVED:
 * - Analog servo reads 0-360° then resets
 * - We need continuous tracking beyond 360°
 * - Starting position = 0° reference
 *
 * HOW IT WORKS:
 * 1. At initialize(), record current raw angle as "zero reference"
 * 2. Track wraparounds (when servo crosses 0°/360° boundary)
 * 3. Calculate continuous angle = (rotations * 360) + (raw - zero)
 * 4. All commands use this continuous angle (can go negative or >360)
 */
@Config
public class NewRampController {

    public static String RAMP_SERVO_NAME = "ramp";
    public static String RAMP_ANALOG_NAME = "ramp_analog";

    // PID Constants (tunable via FTC Dashboard)
    public static double Kp = 0.03;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    // Control parameters
    public static double MAX_POWER = 0.1;           // Maximum servo power
    public static double ANGLE_TOLERANCE = 4.0;     // Degrees - when to stop
    public static double MIN_POWER = 0.05;          // Minimum power to overcome friction
    public static double TIMEOUT_SECONDS = 3.0;     // Timeout for moves

    // Set to true if positive power makes angle go negative
    public static boolean INVERT_OUTPUT = true;

    // Constants for Axon mini servo
    public static double VOLTAGE_TO_DEGREES = 360.0 / 3.3;

    private final CRServo rampServo;
    private final AnalogInput rampAnalog;
    private final ElapsedTime runtime;

    // === CONTINUOUS ANGLE TRACKING ===
    private double zeroRawAngle = 0;      // Raw angle at startup (our "zero" reference)
    private double lastRawAngle = 0;      // Previous raw angle (for wraparound detection)
    private int rotationCount = 0;        // How many full rotations from start
    private boolean initialized = false;

    // PID state
    private double targetAngle = 0;       // Target in continuous angle (can be negative or >360)
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private boolean pidActive = false;
    private double moveStartTime = 0;

    // Velocity tracking
    private double lastAngleForVelocity = 0;
    private double lastVelocityTime = 0;
    private double velocity = 0;

    public NewRampController(LinearOpMode opMode) {
        rampServo = opMode.hardwareMap.get(CRServo.class, RAMP_SERVO_NAME);
        rampAnalog = opMode.hardwareMap.get(AnalogInput.class, RAMP_ANALOG_NAME);
        runtime = new ElapsedTime();

        // Don't reverse - let positive power = positive angle change
        rampServo.setDirection(CRServo.Direction.FORWARD);

        lastTime = runtime.seconds();
        lastVelocityTime = lastTime;
    }

    /**
     * Initialize ramp - sets current position as 0°
     * Call this at the start of teleop/auto
     */
    public void initialize() {
        zeroRawAngle = getRawAngle();
        lastRawAngle = zeroRawAngle;
        rotationCount = 0;
        targetAngle = 0;  // Start position is now 0°
        initialized = true;

        lastAngleForVelocity = 0;
        lastVelocityTime = runtime.seconds();
    }

    /**
     * Check if initialization is complete
     */
    public boolean isInitialized() {
        return initialized;
    }

    /**
     * Update method - handles angle tracking and PID control
     * MUST be called every loop!
     */
    public void update() {
        if (!initialized) {
            initialize();
            return;
        }

        // === UPDATE CONTINUOUS ANGLE TRACKING ===
        double currentRawAngle = getRawAngle();

        // Detect wraparound (servo crossing 0°/360° boundary)
        double delta = currentRawAngle - lastRawAngle;

        // If delta is large negative, servo wrapped from ~360° to ~0° (increasing angle)
        if (delta < -180) {
            rotationCount++;
        }
        // If delta is large positive, servo wrapped from ~0° to ~360° (decreasing angle)
        else if (delta > 180) {
            rotationCount--;
        }

        lastRawAngle = currentRawAngle;

        // === UPDATE VELOCITY ===
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastVelocityTime;

        if (deltaTime > 0.1) {
            double currentAngle = getCurrentAngle();
            double deltaAngle = currentAngle - lastAngleForVelocity;
            velocity = deltaAngle / deltaTime;
            lastAngleForVelocity = currentAngle;
            lastVelocityTime = currentTime;
        }

        // === PID CONTROL ===
        double power = 0;
        if (pidActive) {
            power = calculatePID();

            // Check if target reached
            double currentAngle = getCurrentAngle();
            double error = targetAngle - currentAngle;

            if (Math.abs(error) < ANGLE_TOLERANCE) {
                pidActive = false;
                power = 0;
                integral = 0;
            }

            // Check for timeout
            if (currentTime - moveStartTime > TIMEOUT_SECONDS) {
                pidActive = false;
                power = 0;
                integral = 0;
            }
        }

        // Apply inversion if needed (when positive power makes angle go negative)
        if (INVERT_OUTPUT) {
            power = -power;
        }
        rampServo.setPower(power);
    }

    /**
     * Calculate PID output
     */
    private double calculatePID() {
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;

        if (deltaTime <= 0) return 0;

        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        // Proportional term
        double P = Kp * error;

        // Integral term (with anti-windup)
        integral += error * deltaTime;
        double maxIntegral = MAX_POWER / (Ki + 0.0001);
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double I = Ki * integral;

        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        double D = Kd * derivative;

        // Calculate total output
        double output = P + I + D;

        // Apply minimum power to overcome friction
        if (Math.abs(output) > 0.01 && Math.abs(output) < MIN_POWER) {
            output = Math.signum(output) * MIN_POWER;
        }

        // Clamp output to max power
        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        lastError = error;
        lastTime = currentTime;

        return output;
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
     * Get raw angle from analog feedback (0-360°, resets at 360)
     */
    public double getRawAngle() {
        double voltage = rampAnalog.getVoltage();
        return voltage * VOLTAGE_TO_DEGREES;
    }

    /**
     * Get current continuous angle (relative to start position)
     * This can be negative or greater than 360°
     * Start position = 0°
     */
    public double getCurrentAngle() {
        double currentRawAngle = getRawAngle();

        // Calculate angle relative to zero reference
        double angleFromZero = currentRawAngle - zeroRawAngle;

        // Add full rotations
        double continuousAngle = (rotationCount * 360.0) + angleFromZero;

        return continuousAngle;
    }

    /**
     * Increment ramp angle by specified degrees
     */
    public void incrementAngle(double degrees) {
        targetAngle = getCurrentAngle() + degrees;
        pidActive = true;
        resetPID();
        moveStartTime = runtime.seconds();
    }

    /**
     * Decrement ramp angle by specified degrees
     */
    public void decrementAngle(double degrees) {
        incrementAngle(-degrees);
    }

    /**
     * Set absolute target angle (relative to start position)
     * Example: setTargetAngle(90) moves to 90° from start
     *          setTargetAngle(-45) moves to -45° from start
     */
    public void setTargetAngle(double angle) {
        targetAngle = angle;
        pidActive = true;
        resetPID();
        moveStartTime = runtime.seconds();
    }

    /**
     * Set continuous power to ramp (for manual control, disables PID)
     */
    public void setPower(double power) {
        pidActive = false;
        integral = 0;
        rampServo.setPower(power);
    }

    /**
     * Stop the ramp
     */
    public void stop() {
        pidActive = false;
        integral = 0;
        rampServo.setPower(0);
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
        return !pidActive && Math.abs(targetAngle - getCurrentAngle()) < ANGLE_TOLERANCE;
    }

    /**
     * Get target angle
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get angle error
     */
    public double getAngleError() {
        return targetAngle - getCurrentAngle();
    }

    /**
     * Get the current velocity in degrees per second
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Get the raw voltage from analog feedback
     */
    public double getVoltage() {
        return rampAnalog.getVoltage();
    }

    /**
     * Get the current servo power
     */
    public double getPower() {
        return rampServo.getPower();
    }

    /**
     * Get rotation count (for debugging)
     */
    public int getRotationCount() {
        return rotationCount;
    }

    /**
     * Get zero reference angle (for debugging)
     */
    public double getZeroReference() {
        return zeroRawAngle;
    }
}