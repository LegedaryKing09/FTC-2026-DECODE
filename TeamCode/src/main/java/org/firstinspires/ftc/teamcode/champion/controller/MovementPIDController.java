package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class MovementPIDController {

    // PID coefficients - tunable via FTC Dashboard
    public static double kP = 0.034;
    public static double kI = 0.0;
    public static double kD = 0.0093;
    private double integral = 0;
    private double previousError = 0;
    private double targetDistance = 0;
    private long lastTime = 0;

    // For KI
    public static double MAX_INTEGRAL = 2.0;
    public static double MAX_OUTPUT = 1.0;
    public static double MIN_OUTPUT = -1.0;
    public static double MIN_SPEED = 0.1;

    // Deceleration distance (start slowing down when this close to target)
    public static double DECEL_DISTANCE = 16.0;

    public MovementPIDController() {
    }

    public void setTarget(double distanceInches) {
        targetDistance = distanceInches;
        integral = 0;
        previousError = 0;
        lastTime = System.nanoTime();
    }

    public double update(double currentDistance) {
        double error = targetDistance - currentDistance;
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1_000_000_000.0; // convert to seconds
        lastTime = now;

        // Sanity check on dt
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

        // Apply deceleration profile (slow down near target)
        double maxAllowedSpeed = MAX_OUTPUT;
        if (Math.abs(error) < DECEL_DISTANCE) {
            double decelFactor = Math.abs(error) / DECEL_DISTANCE;
            decelFactor = decelFactor * decelFactor; // Square for smoother decel
            maxAllowedSpeed = MIN_SPEED + (MAX_OUTPUT - MIN_SPEED) * decelFactor;
        }

        // Apply minimum speed to overcome friction
        if (Math.abs(output) > 0.01 && Math.abs(output) < MIN_SPEED) {
            output = Math.signum(output) * MIN_SPEED;
        }

        // Clamp output to deceleration profile
        output = Range.clip(output, -maxAllowedSpeed, maxAllowedSpeed);

        return output;
    }

    public boolean isFinished(double currentDistance, double toleranceInches) {
        double error = targetDistance - currentDistance;
        return Math.abs(error) < toleranceInches;
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