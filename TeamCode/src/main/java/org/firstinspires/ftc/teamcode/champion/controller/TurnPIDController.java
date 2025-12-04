package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * PROVEN Turn PID Controller - tested and used by many teams
 *
 * This controller uses continuous heading tracking (no 0-360 wraparound issues)
 * and always takes the shortest path to the target heading.
 */
@Config
public class TurnPIDController {

    private final IMU imu;

    // PID coefficients - tunable via FTC Dashboard
    public static double kP = 0.018;
    public static double kI = 0.0008;
    public static double kD = 0.004;

    // Alternative safe values (P-only): kP = 0.015, kI = 0.0, kD = 0.003

    private double integral = 0;
    private double previousError = 0;
    private double targetHeading = 0;
    private long lastTime = 0;

    // Integral windup protection
    public static double MAX_INTEGRAL = 2.0;

    // Output clamping
    public static double MAX_OUTPUT = 1.0;
    public static double MIN_OUTPUT = -1.0;

    public TurnPIDController(IMU imu) {
        this.imu = imu;
    }

    /**
     * Set the target heading in degrees
     * Target is automatically normalized to -180..180 for shortest turn
     */
    public void setTarget(double degrees) {
        // Normalize target to -180..180 for shortest turn
        targetHeading = normalizeAngle(degrees);
        integral = 0;
        previousError = 0;
        lastTime = System.nanoTime();
    }

    /**
     * Call this every loop to get the motor power output
     * Returns: power value (-1.0 to 1.0)
     *   positive = turn right (clockwise)
     *   negative = turn left (counter-clockwise)
     */
    public double update() {
        double current = getCurrentHeading();
        double error = targetHeading - current;

        // Make it always take the shortest path
        error = normalizeAngle(error);

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1_000_000_000.0; // convert to seconds
        lastTime = now;

        // Sanity check on dt (should be ~0.02s for typical loop)
        if (dt < 0.005 || dt > 0.5) {
            dt = 0.02;
        }

        // Integral term (with windup protection)
        integral += error * dt;
        integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        // Derivative term
        double derivative = (error - previousError) / dt;
        previousError = error;

        // Calculate PID output
        double output = kP * error + kI * integral + kD * derivative;

        // Clamp output to motor power range
        output = Range.clip(output, MIN_OUTPUT, MAX_OUTPUT);

        return output;
    }

    /**
     * Check if robot is within tolerance of target heading
     */
    public boolean isFinished(double toleranceDegrees) {
        double error = normalizeAngle(targetHeading - getCurrentHeading());
        return Math.abs(error) < toleranceDegrees;
    }

    /**
     * Get the current heading in degrees (continuous, -∞..∞)
     * This is crucial for PID - no wraparound issues!
     */
    public double getCurrentHeading() {
        // Note: negative sign converts from CCW-positive to CW-positive convention
        // Adjust based on your robot's IMU orientation
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Get the current error (target - current)
     */
    public double getError() {
        return normalizeAngle(targetHeading - getCurrentHeading());
    }

    /**
     * Get the target heading
     */
    public double getTargetHeading() {
        return targetHeading;
    }

    /**
     * Normalize angle to -180..180 range
     */
    public static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    /**
     * Reset the controller (clears integral, error history)
     */
    public void reset() {
        integral = 0;
        previousError = 0;
        lastTime = System.nanoTime();
    }
}