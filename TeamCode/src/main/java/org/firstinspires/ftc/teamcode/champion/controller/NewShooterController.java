package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewShooterController {

    private final DcMotorEx shooterMotor;  // Changed to DcMotorEx for velocity API
    private final ElapsedTime runtime;

    // Configurable parameters
    public static double maxPower = 1.0;
    public static boolean reversed = false;

    // Target RPM control
    public static double TARGET_RPM = 1500;
    public static double RPM_INCREMENT = 50.0;
    public static double MIN_RPM = 0.0;
    public static double MAX_RPM = 6000.0;

    // PID gains (tunable via FTC Dashboard)
    public static double kP = 0.65;   // Proportional gain
    public static double kI = 0.0001;   // Integral gain (start small)
    public static double kD = 0.02;  // Derivative gain

    // PID limits
    public static double MAX_INTEGRAL = 5000.0;  // Integral windup limit
    public static double DERIVATIVE_FILTER = 0.7; // Low-pass filter for derivative (0-1)

    // RPM tolerance
    public static double RPM_TOLERANCE = 50.0;

    // Encoder ticks per revolution
    public static double TICKS_PER_REVOLUTION = 28;

    // PID state variables
    private double integralSum = 0;
    private double lastError = 0;
    private double lastDerivative = 0;
    private double lastPidTime = 0;
    private double lastTargetRPM = 0;

    // RPM tracking (for fallback if getVelocity() unavailable)
    private double currentRPM = 0;

    // Shooter state
    private boolean isShootMode = false;
    private double currentTargetRPM;

    public NewShooterController(DcMotor motor) {
        this.runtime = new ElapsedTime();

        // Cast to DcMotorEx for velocity API access
        if (motor instanceof DcMotorEx) {
            this.shooterMotor = (DcMotorEx) motor;
        } else if (motor != null) {
            // Fallback - try to get as DcMotorEx anyway (usually works)
            this.shooterMotor = (DcMotorEx) motor;
        } else {
            this.shooterMotor = null;
        }

        if (shooterMotor != null) {
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // FLOAT is better for flywheels
        }

        lastPidTime = runtime.seconds();
        currentTargetRPM = TARGET_RPM;
    }

    /**
     * Set shooter power directly (for manual control)
     */
    public void setPower(double triggerValue) {
        if (shooterMotor != null) {
            double actualPower = triggerValue * maxPower;
            if (reversed) actualPower = -actualPower;
            shooterMotor.setPower(actualPower);
        }
    }

    /**
     * Activate shoot mode - spinner will try to reach target RPM
     */
    public void startShooting() {
        isShootMode = true;
        // Reset PID state when starting
        integralSum = 0;
        lastError = 0;
        lastDerivative = 0;
        lastPidTime = runtime.seconds();
    }

    /**
     * Stop shoot mode - motor will stop
     */
    public void stopShooting() {
        isShootMode = false;
        integralSum = 0;
        lastError = 0;
        lastDerivative = 0;
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
        }
    }

    /**
     * Toggle shoot mode on/off
     */
    public void toggleShoot() {
        if (isShootMode) {
            stopShooting();
        } else {
            startShooting();
        }
    }

    public void toggleShootDirection() {
        reversed = !reversed;
    }

    /**
     * Increase target RPM
     */
    public void incrementTargetRPM() {
        currentTargetRPM = Math.min(currentTargetRPM + RPM_INCREMENT, MAX_RPM);
    }

    /**
     * Decrease target RPM
     */
    public void decrementTargetRPM() {
        currentTargetRPM = Math.max(currentTargetRPM - RPM_INCREMENT, MIN_RPM);
    }

    /**
     * Set target RPM directly
     */
    public void setTargetRPM(double rpm) {
        currentTargetRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
    }

    /**
     * Get current target RPM
     */
    public double getTargetRPM() {
        return currentTargetRPM;
    }

    /**
     * Check if shooter is at target RPM
     */
    public boolean isAtTargetRPM() {
        if (!isShootMode) return false;
        return Math.abs(currentRPM - currentTargetRPM) <= RPM_TOLERANCE;
    }

    /**
     * Check if shooter is in shoot mode
     */
    public boolean isShootMode() {
        return isShootMode;
    }

    /**
     * Update PID control - call this every loop
     */
    public void update() {
        if (shooterMotor == null) return;

        // Get current RPM from hardware velocity (much more accurate)
        double ticksPerSecond = shooterMotor.getVelocity();
        currentRPM = Math.abs((ticksPerSecond / TICKS_PER_REVOLUTION) * 60.0);

        // If in shoot mode, run PID control
        if (isShootMode && currentTargetRPM > 0) {
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastPidTime;

            // Update PID at high frequency (at least 100Hz)
            if (deltaTime >= 0.01) {
                double error = currentTargetRPM - currentRPM;

                // Proportional term
                double pTerm = kP * error;

                // Integral term with anti-windup
                integralSum += error * deltaTime;
                integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
                double iTerm = kI * integralSum;

                // Derivative term with low-pass filter
                double derivative = (error - lastError) / deltaTime;
                derivative = (DERIVATIVE_FILTER * derivative) + ((1 - DERIVATIVE_FILTER) * lastDerivative);
                double dTerm = kD * derivative;

                // Calculate total PID output
                double power = pTerm + iTerm + dTerm;

                // Clamp power to valid range (only positive for shooter)
                power = Math.max(0.0, Math.min(maxPower, power));

                // Apply direction
                if (reversed) power = -power;

                shooterMotor.setPower(power);

                // Update state for next iteration
                lastError = error;
                lastDerivative = derivative;
                lastPidTime = currentTime;

                // Reset integral if target changed significantly
                if (Math.abs(currentTargetRPM - lastTargetRPM) > 500) {
                    integralSum = 0;
                }
                lastTargetRPM = currentTargetRPM;
            }
        } else if (!isShootMode) {
            // Ensure motor is stopped when not in shoot mode
            shooterMotor.setPower(0);
        }
    }

    /**
     * Get current RPM
     */
    public double getRPM() {
        return currentRPM;
    }

    /**
     * Get RPM error (target - current)
     */
    public double getRPMError() {
        return currentTargetRPM - currentRPM;
    }

    /**
     * Get current power being applied
     */
    public double getCurrentPower() {
        if (shooterMotor == null) return 0.0;
        return shooterMotor.getPower();
    }

    /**
     * Get current encoder position
     */
    public int getEncoderPosition() {
        if (shooterMotor == null) return 0;
        return shooterMotor.getCurrentPosition();
    }

    /**
     * Reset encoder position
     */
    public void resetEncoder() {
        if (shooterMotor != null) {
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currentRPM = 0;
            integralSum = 0;
            lastError = 0;
        }
    }

    /**
     * Check if motor is running
     */
    public boolean isRunning() {
        return getCurrentPower() != 0;
    }

    /**
     * Get integral sum (for debugging/tuning)
     */
    public double getIntegralSum() {
        return integralSum;
    }
}