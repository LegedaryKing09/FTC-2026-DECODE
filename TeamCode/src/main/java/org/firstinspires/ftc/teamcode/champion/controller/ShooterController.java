package org.firstinspires.ftc.teamcode.champion.controller;



import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class ShooterController {
    public static String SHOOTER_NAME1 = "shooter1";
    public static String SHOOTER_NAME2 = "shooter2";

    // Target RPM values instead of power values
    public static double SHOOTER_FULL_RPM = 5000;
    public static double SHOOTER_HALF_RPM = 2500;
    public static double SHOOTER_QUARTER_RPM = 1250;
    public static double SHOOTER_STOP_RPM = 0;

    // Motor specifications
    public static double TICKS_PER_REV = 28;  // Adjust based on your encoder
    public static double WHEEL_DIAMETER_METERS = 0.10795;

    // Velocity control tolerances
    public static double RPM_TOLERANCE = 100; // RPM tolerance for "at target" check

    // PID Constants - Tunable via FTC Dashboard
    // Conservative values to prevent overshoot
    public static double kP = 0.85;    // Power per RPM error - reduced to prevent overshoot
    public static double kI = 0.0;    // Start with 0, add later for steady-state correction
    public static double kD = 0.02;   // Derivative for stability - reduced for smoother response

    // PID limits
    public static double MAX_INTEGRAL = 2000;  // Integral windup limit in RPM*seconds
    public static double DERIVATIVE_FILTER_GAIN = 0.7; // 0-1, higher = less filtering

    private enum ShooterMode {
        VELOCITY_CONTROL, STOP
    }

    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;
    private ShooterMode shooterMode = ShooterMode.STOP;
    private double targetRPM = 0;

    // PID variables
    private double lastError = 0;
    private double integralSum = 0;
    private double lastDerivative = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastPidTime = 0;
    private double lastTargetRPM = 0;

    // PID output tracking
    private double pidOutput = 0;

    public ShooterController(LinearOpMode opMode) {
        shooter1 = opMode.hardwareMap.get(DcMotorEx.class, SHOOTER_NAME1);
        shooter2 = opMode.hardwareMap.get(DcMotorEx.class, SHOOTER_NAME2);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoder but run in direct power mode for both motors
        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Direct power control

        // Shooter2 runs without encoder
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize timers
        pidTimer.reset();
    }

    // Get raw encoder position from shooter1
    public int getEncoderPosition() {
        return shooter1.getCurrentPosition();
    }

    // Get raw encoder velocity from shooter1 (ticks per second)
    public double getEncoderVelocity() {
        return shooter1.getVelocity();
    }

    public double getShooterRPM() {
        // Get velocity in ticks per second from shooter1 encoder and convert to RPM
        double ticksPerSecond = shooter1.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    // Shooter wheel velocity in meters per second
    public double getShooterMPS() {
        double rpm = getShooterRPM();
        double revsPerSecond = rpm / 60.0;
        double circumference = Math.PI * WHEEL_DIAMETER_METERS;
        return revsPerSecond * circumference;
    }

    // Get the target RPM
    public double getTargetRPM() {
        return targetRPM;
    }

    // Check if shooter is at target RPM within tolerance
    public boolean isAtTargetRPM() {
        return Math.abs(getShooterRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    // Main PID update function - should be called regularly from opMode loop
    public void updatePID() {
        if (shooterMode != ShooterMode.STOP && targetRPM > 0) {
            double currentTime = pidTimer.seconds();
            double deltaTime = currentTime - lastPidTime;

            if (deltaTime > 0.005) { // Update at max 200Hz for better response
                double currentRPM = getShooterRPM();
                double error = targetRPM - currentRPM;

                // NORMAL PID CONTROL
                if (shooterMode == ShooterMode.VELOCITY_CONTROL) {
                    // Proportional term - immediate response to error
                    double pTerm = kP * error;

                    // Integral term - accumulates error over time to eliminate steady-state
                    integralSum += error * deltaTime;
                    // Anti-windup: limit integral accumulation
                    if (Math.abs(integralSum) > MAX_INTEGRAL) {
                        integralSum = Math.signum(integralSum) * MAX_INTEGRAL;
                    }
                    double iTerm = kI * integralSum;

                    // Derivative term - predicts future error based on rate of change
                    double derivative = 0;
                    if (deltaTime > 0) {
                        derivative = (error - lastError) / deltaTime;
                        // Low-pass filter to reduce noise
                        derivative = (DERIVATIVE_FILTER_GAIN * derivative) +
                                ((1 - DERIVATIVE_FILTER_GAIN) * lastDerivative);
                    }
                    double dTerm = kD * derivative;

                    // Calculate total PID output as motor power adjustment
                    double pidCorrection = pTerm + iTerm + dTerm;

                    // Total motor power = PID correction only (no feedforward)
                    double motorPower = pidCorrection;

                    // Aggressive clamping - allow full power when needed
                    motorPower = Math.max(0.0, Math.min(1.0, motorPower));

                    // Apply power to both motors
                    shooter1.setPower(motorPower);
                    shooter2.setPower(-motorPower);

                    // Store for telemetry
                    pidOutput = pidCorrection * SHOOTER_FULL_RPM; // Convert to RPM units for display

                    // Update derivative state
                    lastDerivative = derivative;
                }

                // Update state for next iteration
                lastError = error;
                lastPidTime = currentTime;
            }
        }
    }

    private void setTargetRPM(double rpm) {
        targetRPM = rpm;

        if (rpm <= 0) {
            // Stop both motors
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooterMode = ShooterMode.STOP;

            // Reset PID state
            integralSum = 0;
            lastError = 0;
            lastDerivative = 0;
            pidOutput = 0;
        } else {
            shooterMode = ShooterMode.VELOCITY_CONTROL;

            // Reset integral when target changes significantly
            // This prevents old integral from interfering with new target
            if (Math.abs(rpm - lastTargetRPM) > 500) {
                integralSum = 0;
                lastError = 0;
            }

            // Set initial power to 0 - let PID handle everything
            // Apply initial power to get motors spinning immediately
            shooter1.setPower(0);
            shooter2.setPower(0);

            lastTargetRPM = targetRPM;
        }
    }

    public void shooterFull() {
        setTargetRPM(SHOOTER_FULL_RPM);
    }

    public void shooterHalf() {
        setTargetRPM(SHOOTER_HALF_RPM);
    }

    public void shooterQuarter() {
        setTargetRPM(SHOOTER_QUARTER_RPM);
    }

    public void shooterStop() {
        setTargetRPM(SHOOTER_STOP_RPM);
    }

    // Set custom RPM target
    public void setShooterRPM(double rpm) {
        setTargetRPM(rpm);
    }

    // Legacy method - now sets equivalent RPM
    public void setShooterPower(double power) {
        double rpm = power * SHOOTER_FULL_RPM;
        setTargetRPM(rpm);
    }

    public double getShooterPower() {
        return shooter1.getPower();
    }

    public double getShooter2Power() {
        return shooter2.getPower();
    }

    public boolean isDoingShooter() {
        return shooterMode != ShooterMode.STOP;
    }

    // Get current RPM error for debugging
    public double getRPMError() {
        return targetRPM - getShooterRPM();
    }

    // Get PID terms for tuning (debugging)
    public double getPIDOutput() {
        return pidOutput;
    }

    public double getIntegralSum() {
        return integralSum;
    }

    public double getLastPTerm() {
        return kP * lastError;
    }

    public double getLastITerm() {
        return kI * integralSum;
    }

    public double getLastDTerm() {
        return kD * lastDerivative;
    }

    // Add telemetry data to the telemetry object
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter Status", shooterMode);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Current RPM", "%.0f", getShooterRPM());
        telemetry.addData("RPM Error", "%.0f", getRPMError());
        telemetry.addData("At Target", isAtTargetRPM() ? "YES" : "NO");
        telemetry.addData("Shooter1 Power", "%.3f", getShooterPower());
        telemetry.addData("Shooter2 Power", "%.3f", getShooter2Power());
        telemetry.addData("Encoder Velocity", "%.1f tps", getEncoderVelocity());

        // PID debugging telemetry
        telemetry.addData("PID Output", "%.1f RPM", pidOutput);
        telemetry.addData("PID P-term", "%.4f", getLastPTerm());
        telemetry.addData("PID I-term", "%.4f", getLastITerm());
        telemetry.addData("PID D-term", "%.4f", getLastDTerm());
        telemetry.addData("Integral Sum", "%.1f", integralSum);
    }

    // Get telemetry data as string (alternative method)
    @SuppressLint("DefaultLocale")
    public String getTelemetryData() {
        return String.format("Target: %.0f RPM | Current: %.0f RPM | Error: %.0f | At Target: %s",
                targetRPM, getShooterRPM(), getRPMError(), isAtTargetRPM() ? "YES" : "NO");
    }
}