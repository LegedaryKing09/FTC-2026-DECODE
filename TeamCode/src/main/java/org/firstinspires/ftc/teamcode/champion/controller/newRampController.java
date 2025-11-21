package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class newRampController {

    public static String RAMP_SERVO_NAME = "ramp";
    public static String RAMP_ANALOG_NAME = "ramp_analog";

    // PID Constants (tunable via FTC Dashboard)
    public static double kP = 0.015;
    public static double kI = 0.0005;
    public static double kD = 0.002;
    public static double MAX_POWER = 0.8;
    public static double MIN_POWER = 0.05;
    public static double INTEGRAL_LIMIT = 50.0;

    // Angle range (0-360 degrees for full rotation servo)
    public static double MIN_ANGLE_DEGREES = 0.0;
    public static double MAX_ANGLE_DEGREES = 360.0;

    // Position tolerance for reaching target (in degrees)
    public static double POSITION_TOLERANCE_DEGREES = 1.0;

    // Movement timeout (milliseconds)
    public static long MOVEMENT_TIMEOUT_MS = 3000;

    // Common preset angles (adjust these for your robot)
    public static double ANGLE_STORED = 0.0;      // Ramp stored position
    public static double ANGLE_LOW = 30.0;        // Low shot angle
    public static double ANGLE_MEDIUM = 45.0;     // Medium shot angle
    public static double ANGLE_HIGH = 60.0;       // High shot angle
    public static double ANGLE_MAX = 85.0;        // Maximum safe angle

    // Constants for Axon mini servo
    private static final double VOLTAGE_TO_DEGREES = 360.0 / 3.3;

    private final CRServo rampServo;
    private final AnalogInput rampAnalog;
    private final ElapsedTime runtime;

    // Movement state
    private boolean isMoving = false;
    private double targetAngle = 0.0;
    private long movementStartTime = 0;

    // PID variables
    private double lastError = 0;
    private double integralSum = 0;
    private double lastPIDTime = 0;

    // Velocity tracking
    private double lastPosition = 0;
    private double velocity = 0;
    private double lastTime = 0;

    // Final error tracking
    private double finalError = 0;

    public newRampController(LinearOpMode opMode) {
        rampServo = opMode.hardwareMap.get(CRServo.class, RAMP_SERVO_NAME);
        rampAnalog = opMode.hardwareMap.get(AnalogInput.class, RAMP_ANALOG_NAME);
        runtime = new ElapsedTime();

        // Initialize to current position (now reads fresh data)
        targetAngle = getCurrentAngle();
        lastPosition = targetAngle;
        lastTime = runtime.seconds();
        lastPIDTime = lastTime;
    }

    /**
     * CRITICAL: Must be called in every loop iteration to update servo movement
     * Similar to ShooterController.updatePID()
     */
    public void update() {
        // Read current position (fresh from sensor)
        double currentPosition = getCurrentAngle();

        // Calculate velocity (degrees per second)
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;

        if (deltaTime > 0) {
            double deltaPosition = currentPosition - lastPosition;

            // Handle wraparound (crossing 0/360 boundary)
            if (deltaPosition > 180) {
                deltaPosition -= 360;
            } else if (deltaPosition < -180) {
                deltaPosition += 360;
            }

            velocity = deltaPosition / deltaTime;
        }

        // Update last values
        lastPosition = currentPosition;
        lastTime = currentTime;

        // If not moving, stop here
        if (!isMoving) {
            return;
        }

        // Calculate shortest path to target
        double error = targetAngle - currentPosition;

        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // Check if we've reached the target
        if (Math.abs(error) < POSITION_TOLERANCE_DEGREES) {
            rampServo.setPower(0);
            isMoving = false;
            finalError = error;
            // Reset PID variables
            integralSum = 0;
            lastError = 0;
            return;
        }

        // Check for timeout
        if (System.currentTimeMillis() - movementStartTime > MOVEMENT_TIMEOUT_MS) {
            rampServo.setPower(0);
            isMoving = false;
            finalError = error;
            // Reset PID variables
            integralSum = 0;
            lastError = 0;
            return;
        }

        // PID Control
        double pidDeltaTime = currentTime - lastPIDTime;

        if (pidDeltaTime > 0) {
            // Proportional term
            double pTerm = kP * error;

            // Integral term with anti-windup
            integralSum += error * pidDeltaTime;
            integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
            double iTerm = kI * integralSum;

            // Derivative term
            double derivative = (error - lastError) / pidDeltaTime;
            double dTerm = kD * derivative;

            // Calculate total power
            double power = pTerm + iTerm + dTerm;

            // Apply minimum power to overcome friction (only when error is significant)
            if (Math.abs(error) > POSITION_TOLERANCE_DEGREES) {
                if (power > 0 && power < MIN_POWER) {
                    power = MIN_POWER;
                } else if (power < 0 && power > -MIN_POWER) {
                    power = -MIN_POWER;
                }
            }

            // Clamp power to maximum
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            // Set servo power
            rampServo.setPower(power);

            lastError = error;
            lastPIDTime = currentTime;
        }
    }

    /**
     * Set the ramp to a specific angle in degrees
     * @param angleDegrees Target angle (0-360 degrees)
     */
    public void setAngle(double angleDegrees) {
        // Clamp angle to valid range
        angleDegrees = Math.max(MIN_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, angleDegrees));

        targetAngle = angleDegrees;
        isMoving = true;
        movementStartTime = System.currentTimeMillis();

        // Reset PID variables
        integralSum = 0;
        lastError = 0;
        lastPIDTime = runtime.seconds();
    }

    /**
     * Get the current ramp angle from analog feedback
     * ALWAYS reads fresh data from the sensor
     * @return Current angle in degrees (0-360)
     */
    public double getCurrentAngle() {
        double voltage = rampAnalog.getVoltage();
        return voltage * VOLTAGE_TO_DEGREES;
    }

    /**
     * Get the target angle
     * @return Target angle in degrees
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get the current velocity in degrees per second
     * @return Velocity (degrees/sec)
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Get the current position (0.0-1.0)
     * @return Normalized position
     */
    public double getPosition() {
        return getCurrentAngle() / 360.0;
    }

    /**
     * Get the raw voltage from analog feedback
     * @return Voltage (0-3.3V)
     */
    public double getVoltage() {
        return rampAnalog.getVoltage();
    }

    /**
     * Get the current servo power
     * @return Power (-1.0 to 1.0)
     */
    public double getPower() {
        return rampServo.getPower();
    }

    /**
     * Check if the ramp is currently moving
     * @return true if moving to target
     */
    public boolean isMoving() {
        return isMoving;
    }

    /**
     * Check if the ramp has reached the target angle
     * @return true if at target (within tolerance)
     */
    public boolean isAtTargetAngle() {
        double error = getAngularError(getCurrentAngle(), targetAngle);
        return Math.abs(error) <= POSITION_TOLERANCE_DEGREES;
    }

    /**
     * Get the final error from the last completed movement
     * @return Final error in degrees
     */
    public double getFinalError() {
        return finalError;
    }

    /**
     * Increment the target angle by a specified amount
     * @param degrees Amount to increase (positive value)
     */
    public void incrementAngle(double degrees) {
        double currentAngle = getCurrentAngle();
        setAngle(currentAngle + degrees);
    }

    /**
     * Decrement the target angle by a specified amount
     * @param degrees Amount to decrease (positive value)
     */
    public void decrementAngle(double degrees) {
        incrementAngle(-degrees);
    }

    /**
     * Stop the ramp immediately
     */
    public void stop() {
        rampServo.setPower(0);
        isMoving = false;
        targetAngle = getCurrentAngle();  // Set target to current position
        // Reset PID variables
        integralSum = 0;
        lastError = 0;
    }

    /**
     * Get the angular error between current and target
     * Positive = need to rotate clockwise
     * Negative = need to rotate counter-clockwise
     */
    private double getAngularError(double current, double target) {
        double error = target - current;

        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    // ========== PRESET ANGLE METHODS ==========

    public void setToStored() {
        setAngle(ANGLE_STORED);
    }

    public void setToLow() {
        setAngle(ANGLE_LOW);
    }

    public void setToMedium() {
        setAngle(ANGLE_MEDIUM);
    }

    public void setToHigh() {
        setAngle(ANGLE_HIGH);
    }

    public void setToMax() {
        setAngle(ANGLE_MAX);
    }

    // ========== UTILITY METHODS ==========

    /**
     * Check if ramp is at a specific angle (within tolerance)
     * @param targetAngle Angle to check
     * @param toleranceDegrees Tolerance in degrees
     * @return true if within tolerance
     */
    public boolean isAtAngle(double targetAngle, double toleranceDegrees) {
        double error = Math.abs(getAngularError(getCurrentAngle(), targetAngle));
        return error <= toleranceDegrees;
    }

    /**
     * Get the error between current angle and target angle
     * @return Error in degrees (positive if target is ahead)
     */
    public double getAngleError() {
        return getAngularError(getCurrentAngle(), targetAngle);
    }

    /**
     * Get the absolute error between current and target
     * @return Absolute error in degrees
     */
    public double getAbsoluteAngleError() {
        return Math.abs(getAngleError());
    }

    /**
     * Get the P term of the PID controller
     * @return Proportional term
     */
    public double getPTerm() {
        return kP * getAngleError();
    }

    /**
     * Get the I term of the PID controller
     * @return Integral term
     */
    public double getITerm() {
        return kI * integralSum;
    }

    /**
     * Get the D term of the PID controller
     * @return Derivative term
     */
    public double getDTerm() {
        return kD * lastError;
    }

    /**
     * Wait for the ramp to reach its target angle
     * WARNING: This is a blocking call, use carefully
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return true if target reached, false if timeout
     */
    public boolean waitForTarget(long timeoutMs) {
        long startTime = System.currentTimeMillis();

        while (isMoving && (System.currentTimeMillis() - startTime) < timeoutMs) {
            update();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                return false;
            }
        }

        return isAtTargetAngle();
    }
}