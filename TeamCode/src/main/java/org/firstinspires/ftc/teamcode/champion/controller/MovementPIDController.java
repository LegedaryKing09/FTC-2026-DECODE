package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class MovementPIDController {

    // PID coefficients
    public static double kP = 0.034;  // responds to current error
    public static double kI = 0.0;     // responds to accumulated error
    public static double kD = 0.0093;  // responds to rate of error change

    // Feedforward coefficient : helps overcome static friction and provides base power
    public static double kF = 0.05;    // Base power when moving (helps robot start moving)

    private double integral = 0;        // Accumulated error over time
    private double previousError = 0;   // Error from last update (for derivative)
    private double targetDistance = 0;  // Where we want to go (inches)
    private long lastTime = 0;          // Last update time (for calculating dt)

    // For KI
    public static double MAX_INTEGRAL = 2.0;  // Prevents integral windup (runaway)
    public static double MAX_OUTPUT = 1.0;     // Maximum motor power (100%)
    public static double MIN_SPEED = 0.1;      // Minimum power to overcome friction

    // Deceleration distance (start slowing down when this close to target)
    public static double DECEL_DISTANCE = 16.0;  // Begin ramping down 16 inches from target

    public void setTarget(double distanceInches) {
        targetDistance = distanceInches;  // Store where we want to go
        integral = 0;                      // Clear accumulated error
        previousError = 0;                 // Clear previous error
        lastTime = System.nanoTime();      // Start timing from now
    }

    public double update(double currentDistance) {
        // Calculate how far we still need to go
        double error = targetDistance - currentDistance;

        // Get current time and calculate time since last update
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1_000_000_000.0; // Convert nanoseconds to seconds
        lastTime = now;

        if (dt < 0.005 || dt > 0.5) {
            dt = 0.02;  // Use default 20ms if dt is weird
        }

        // helps eliminate steady-state error (when robot settles short of target)
        integral += error * dt;  // Add error × time to running total
        integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);  // Prevent windup

        // dampens oscillations by resisting rapid changes
        double derivative = (error - previousError) / dt;  // How fast error is changing
        previousError = error;  // Save for next calculation

        // This provides a baseline power to overcome friction
        // Sign of error determines direction
        double feedforward = Math.signum(error) * kF;

        // Calculate total PID output
        // P term: push harder when far from target
        // I term: fix persistent small errors
        // D term: slow down when approaching target
        // F term: baseline power to start/maintain motion
        double output = kP * error + kI * integral + kD * derivative + feedforward;

        // DECELERATION PROFILE: Slow down smoothly as we approach target
        double maxAllowedSpeed = MAX_OUTPUT;
        if (Math.abs(error) < DECEL_DISTANCE) {
            // Calculate what fraction of DECEL_DISTANCE we are from target
            double decelFactor = Math.abs(error) / DECEL_DISTANCE;  // 0.0 to 1.0
            decelFactor = decelFactor * decelFactor;  // Square it for smoother curve (parabolic)

            // Ramp from MIN_SPEED (at target) to MAX_OUTPUT (at DECEL_DISTANCE away)
            maxAllowedSpeed = MIN_SPEED + (MAX_OUTPUT - MIN_SPEED) * decelFactor;
        }

        // Apply minimum speed to overcome static friction
        if (Math.abs(output) > 0.01 && Math.abs(output) < MIN_SPEED) {
            output = Math.signum(output) * MIN_SPEED;  // Keep direction, set minimum magnitude
        }

        // Clamp output to deceleration profile limits
        output = Range.clip(output, -maxAllowedSpeed, maxAllowedSpeed);

        return output;  // Send this power to motors
    }

    public boolean isFinished(double currentDistance, double toleranceInches) {
        double error = targetDistance - currentDistance;
        return Math.abs(error) < toleranceInches;  // Are we close enough?
    }

    public double getError(double currentDistance) {
        return targetDistance - currentDistance;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        lastTime = System.nanoTime();
    }
}