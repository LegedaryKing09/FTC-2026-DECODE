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

    // Incremental control settings (matching RampTest)
    public static double INCREMENT_POWER = 0.1;     // Power for increments
    public static long INCREMENT_TIME_MS = 100;      // How long to pulse (milliseconds)

    // Constants for Axon mini servo
    private static final double VOLTAGE_TO_DEGREES = 360.0 / 3.3;

    private final CRServo rampServo;
    private final AnalogInput rampAnalog;
    private final ElapsedTime runtime;

    // Increment state tracking
    private long incrementStartTime = 0;
    private boolean isIncrementing = false;
    private double incrementDirection = 0;

    // Velocity tracking
    private double lastAngle = 0;
    private double lastTime = 0;
    private double velocity = 0;

    public NewRampController(LinearOpMode opMode) {
        rampServo = opMode.hardwareMap.get(CRServo.class, RAMP_SERVO_NAME);
        rampAnalog = opMode.hardwareMap.get(AnalogInput.class, RAMP_ANALOG_NAME);
        runtime = new ElapsedTime();

        // REVERSE the servo direction since only negative power works
        rampServo.setDirection(CRServo.Direction.REVERSE);

        lastAngle = getCurrentAngle();
        lastTime = runtime.seconds();
    }

    /**
     * Update method - handles increment timing and velocity calculation
     * MUST be called every loop!
     */
    public void update() {
        // Update velocity
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;

        if (deltaTime > 0.1) {
            double currentAngle = getCurrentAngle();
            double deltaAngle = currentAngle - lastAngle;

            // Handle wraparound
            if (deltaAngle > 180) deltaAngle -= 360;
            if (deltaAngle < -180) deltaAngle += 360;

            velocity = deltaAngle / deltaTime;
            lastAngle = currentAngle;
            lastTime = currentTime;
        }

        // Handle increment timing
        double power = 0;
        if (isIncrementing) {
            if (System.currentTimeMillis() - incrementStartTime < INCREMENT_TIME_MS) {
                power = incrementDirection;
            } else {
                isIncrementing = false;
                power = 0;
            }
        }

        rampServo.setPower(power);
    }

    /**
     * Increment ramp angle (pulse upward)
     */
    public void incrementAngle(double degrees) {
        // Note: degrees parameter not used for CR servo
        // We just pulse with fixed power and time
        isIncrementing = true;
        incrementDirection = INCREMENT_POWER;
        incrementStartTime = System.currentTimeMillis();
    }

    /**
     * Decrement ramp angle (pulse downward)
     */
    public void decrementAngle(double degrees) {
        // Note: degrees parameter not used for CR servo
        // We just pulse with fixed power and time
        isIncrementing = true;
        incrementDirection = -INCREMENT_POWER;
        incrementStartTime = System.currentTimeMillis();
    }

    /**
     * Set continuous power to ramp (for manual control)
     */
    public void setPower(double power) {
        isIncrementing = false;
        rampServo.setPower(power);
    }

    /**
     * Stop the ramp
     */
    public void stop() {
        isIncrementing = false;
        rampServo.setPower(0);
    }

    /**
     * Get current angle from analog feedback
     */
    public double getCurrentAngle() {
        double voltage = rampAnalog.getVoltage();
        return voltage * VOLTAGE_TO_DEGREES;
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
     * Check if currently incrementing
     */
    public boolean isIncrementing() {
        return isIncrementing;
    }

    /**
     * Get increment direction (for telemetry)
     */
    public String getIncrementDirection() {
        if (!isIncrementing) return "NONE";
        return incrementDirection > 0 ? "UP" : "DOWN";
    }
}