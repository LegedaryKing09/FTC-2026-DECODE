package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewRampController {

    public static String RAMP_SERVO_NAME = "ramp";
    public static String RAMP_ANALOG_NAME = "ramp_analog";

    // PID Constants (tunable via FTC Dashboard)
    public static double RAMP_Kp = 0.02;
    public static double RAMP_Ki = 0.0;
    public static double RAMP_Kd = 0.001;

    // Control parameters (tunable via FTC Dashboard)
    public static double RAMP_MAX_POWER = 0.5;           // Maximum servo power
    public static double RAMP_ANGLE_TOLERANCE = 2.0;     // Degrees - when to stop
    public static double RAMP_MIN_POWER = 0.05;          // Minimum power to overcome friction
    public static double RAMP_TIMEOUT_SECONDS = 3.0;     // Timeout for moves
    public static double RAMP_INIT_ANGLE = 0.0;          // Initialization target angle

    // Constants for Axon mini servo
    private static final double VOLTAGE_TO_DEGREES = 360.0 / 3.3;

    private final CRServo rampServo;
    private final AnalogInput rampAnalog;
    private final ElapsedTime runtime;

    // PID state
    private double targetAngle = 0;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private boolean pidActive = false;
    private double moveStartTime = 0;

    // Velocity tracking
    private double lastAngle = 0;
    private double lastVelocityTime = 0;
    private double velocity = 0;

    // Initialization state
    private boolean isInitialized = false;

    public NewRampController(LinearOpMode opMode) {
        rampServo = opMode.hardwareMap.get(CRServo.class, RAMP_SERVO_NAME);
        rampAnalog = opMode.hardwareMap.get(AnalogInput.class, RAMP_ANALOG_NAME);
        runtime = new ElapsedTime();

        // REVERSE the servo direction since only negative power works
        rampServo.setDirection(CRServo.Direction.REVERSE);

        lastAngle = getCurrentAngle();
        lastTime = runtime.seconds();
        lastVelocityTime = lastTime;
        targetAngle = getCurrentAngle();
    }

    /**
     * Initialize ramp to configured angle
     * Call this during initialization or at the start of autonomous/teleop
     */
    public void initialize() {
        targetAngle = RAMP_INIT_ANGLE;
        pidActive = true;
        resetPID();
        moveStartTime = runtime.seconds();
        isInitialized = false;
    }

    /**
     * Check if initialization is complete
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Update method - handles PID control and velocity calculation
     * MUST be called every loop!
     */
    public void update() {
        // Update velocity
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastVelocityTime;

        if (deltaTime > 0.1) {
            double currentAngle = getCurrentAngle();
            double deltaAngle = currentAngle - lastAngle;

            // Handle wraparound
            if (deltaAngle > 180) deltaAngle -= 360;
            if (deltaAngle < -180) deltaAngle += 360;

            velocity = deltaAngle / deltaTime;
            lastAngle = currentAngle;
            lastVelocityTime = currentTime;
        }

        // Handle PID control
        double power = 0;
        if (pidActive) {
            power = calculatePID();

            // Check if target reached
            double currentAngle = getCurrentAngle();
            double error = getShortestAngleError(targetAngle, currentAngle);

            if (Math.abs(error) < RAMP_ANGLE_TOLERANCE) {
                pidActive = false;
                power = 0;
                integral = 0;

                // Mark as initialized if we were initializing
                if (!isInitialized && Math.abs(targetAngle - RAMP_INIT_ANGLE) < RAMP_ANGLE_TOLERANCE) {
                    isInitialized = true;
                }
            }

            // Check for timeout
            if (currentTime - moveStartTime > RAMP_TIMEOUT_SECONDS) {
                pidActive = false;
                power = 0;
                integral = 0;
            }
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
        double error = getShortestAngleError(targetAngle, currentAngle);

        // Proportional term
        double P = RAMP_Kp * error;

        // Integral term (with anti-windup)
        integral += error * deltaTime;
        // Limit integral to prevent windup
        double maxIntegral = RAMP_MAX_POWER / (RAMP_Ki + 0.0001); // Avoid division by zero
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double I = RAMP_Ki * integral;

        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        double D = RAMP_Kd * derivative;

        // Calculate total output
        double output = P + I + D;

        // Apply minimum power to overcome friction
        if (Math.abs(output) > 0.01 && Math.abs(output) < RAMP_MIN_POWER) {
            output = Math.signum(output) * RAMP_MIN_POWER;
        }

        // Clamp output to max power
        output = Math.max(-RAMP_MAX_POWER, Math.min(RAMP_MAX_POWER, output));

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
     * Increment ramp angle by specified degrees
     */
    public void incrementAngle(double degrees) {
        targetAngle = getCurrentAngle() + degrees;

        // Wrap target angle to [0, 360]
        while (targetAngle >= 360) targetAngle -= 360;
        while (targetAngle < 0) targetAngle += 360;

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
     * Set absolute target angle
     */
    public void setTargetAngle(double angle) {
        targetAngle = angle;

        // Wrap target angle to [0, 360]
        while (targetAngle >= 360) targetAngle -= 360;
        while (targetAngle < 0) targetAngle += 360;

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
        return !pidActive && Math.abs(getShortestAngleError(targetAngle, getCurrentAngle())) < RAMP_ANGLE_TOLERANCE;
    }

    /**
     * Get current angle from analog feedback
     */
    public double getCurrentAngle() {
        double voltage = rampAnalog.getVoltage();
        return voltage * VOLTAGE_TO_DEGREES;
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
        return getShortestAngleError(targetAngle, getCurrentAngle());
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
     * Get PID components (for debugging)
     */
    public double getProportional() {
        return RAMP_Kp * getAngleError();
    }

    public double getIntegral() {
        return RAMP_Ki * integral;
    }

    public double getDerivative() {
        return RAMP_Kd * (getAngleError() - lastError);
    }
}