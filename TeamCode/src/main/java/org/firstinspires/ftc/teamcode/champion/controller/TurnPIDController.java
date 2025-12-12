package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Config
public class TurnPIDController {
    private final GoBildaPinpointDriver odo;

    // PID coefficients - tunable via FTC Dashboard
    public static double kP = 0.018;   // Proportional gain
    public static double kI = 0.0008;  // Integral gain
    public static double kD = 0.004;   // Derivative gain

    // Feedforward coefficient
    public static double kF = 0.03;    // Base turning power (helps overcome static friction)

    private double integral = 0;        // Accumulated error
    private double previousError = 0;   // Previous error for derivative
    private double targetHeading = 0;   // Target angle in degrees
    private long lastTime = 0;          // Timestamp for dt calculation

    // Integral windup protection
    public static double MAX_INTEGRAL = 2.0;  // Cap on integral accumulation

    // Output clamping
    public static double MAX_OUTPUT = 1.0;   // Max turning power
    public static double MIN_OUTPUT = -1.0;  // Min turning power

    public TurnPIDController(GoBildaPinpointDriver odo) {
        this.odo = odo;  // Store reference to odometry computer
    }

    public void setTarget(double degrees) {
        // Normalize target to -180..180 for shortest turn path
        targetHeading = normalizeAngle(degrees);
        integral = 0;               // Reset accumulated error
        previousError = 0;          // Reset derivative calculation
        lastTime = System.nanoTime();  // Start timing
    }

    public double update() {
        // Get current heading from Pinpoint's IMU
        double current = getCurrentHeading();

        // Calculate error (how far we need to turn)
        double error = targetHeading - current;

        // Normalize error to -180..180 to always take shortest path
        error = normalizeAngle(error);

        // Calculate time since last update
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1_000_000_000.0; // nanoseconds → seconds
        lastTime = now;

        // reject unrealistic time intervals
        if (dt < 0.005 || dt > 0.5) {
            dt = 0.02;  // Default to 20ms if something weird happened
        }

        // INTEGRAL TERM: Accumulate error over time
        integral += error * dt;
        // Clamp integral to prevent windup (integral getting too large)
        integral = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        // DERIVATIVE TERM: Rate of change of error
        double derivative = (error - previousError) / dt;
        previousError = error;  // Store for next iteration

        // Base power to initiate/maintain rotation
        double feedforward = Math.signum(error) * kF;

        // Calculate total PID+F output
        // P: corrects current error
        // I: eliminates steady-state error
        // D: dampens oscillation
        // F: overcomes friction
        double output = kP * error + kI * integral + kD * derivative + feedforward;

        // Clamp output to valid motor power range
        output = Range.clip(output, MIN_OUTPUT, MAX_OUTPUT);

        return output;
    }

    public boolean isFinished(double toleranceDegrees) {
        double error = normalizeAngle(targetHeading - getCurrentHeading());
        return Math.abs(error) < toleranceDegrees;
    }

    public double getCurrentHeading() {
        // Get heading from Pinpoint's internal IMU
        return odo.getHeading(AngleUnit.DEGREES);
    }

    public double getError() {
        return normalizeAngle(targetHeading - getCurrentHeading());
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public static double normalizeAngle(double angle) {
        // Keep subtracting/adding 360 until angle is in range
        while (angle > 180) angle -= 360;   // If too positive, wrap around
        while (angle <= -180) angle += 360; // If too negative, wrap around
        return angle;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        lastTime = System.nanoTime();
    }
}