package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewShooterController {

    private final DcMotorEx shooterMotorFirst;  // Motor with encoder
    private final DcMotorEx shooterMotorSecond; // motor without encoder
    private final ElapsedTime runtime;

    // Configurable parameters
    public static double maxPower = 1.0;
    public static boolean reversed = false;

    // parameter to control if second motor is reversed relative to first
    public static boolean secondMotorReversed = true;

    // Target RPM control
    public static double TARGET_RPM = 1500;
    public static double RPM_INCREMENT = 50.0;
    public static double MIN_RPM = 0.0;
    public static double MAX_RPM = 6000.0;

    // PID gains (tunable via FTC Dashboard)
    public static double kP = 0.65;
    public static double kI = 0.0001;
    public static double kD = 0.02;

    // PID limits
    public static double MAX_INTEGRAL = 5000.0;
    public static double DERIVATIVE_FILTER = 0.7;

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

    // RPM tracking
    private double currentRPM = 0;

    // Shooter state
    private boolean isShootMode = false;
    private double currentTargetRPM;

    public NewShooterController(DcMotor motorFirst, DcMotor motorSecond) {
        this.runtime = new ElapsedTime();

        // Initialize first motor (with encoder)
        if (motorFirst instanceof DcMotorEx) {
            this.shooterMotorFirst = (DcMotorEx) motorFirst;
        } else if (motorFirst != null) {
            this.shooterMotorFirst = (DcMotorEx) motorFirst;
        } else {
            this.shooterMotorFirst = null;
        }

        // Initialize second motor (without encoder)
        if (motorSecond instanceof DcMotorEx) {
            this.shooterMotorSecond = (DcMotorEx) motorSecond;
        } else if (motorSecond != null) {
            this.shooterMotorSecond = (DcMotorEx) motorSecond;
        } else {
            this.shooterMotorSecond = null;
        }

        // Configure first motor
        if (shooterMotorFirst != null) {
            shooterMotorFirst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotorFirst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotorFirst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Configure second motor
        if (shooterMotorSecond != null) {
            shooterMotorSecond.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotorSecond.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        lastPidTime = runtime.seconds();
        currentTargetRPM = TARGET_RPM;
    }

    /**
     * Set shooter power directly (for manual control)
     */
    public void setPower(double triggerValue) {
        double actualPower = triggerValue * maxPower;
        if (reversed) actualPower = -actualPower;

        // Apply power to first motor
        if (shooterMotorFirst != null) {
            shooterMotorFirst.setPower(actualPower);
        }

        // Apply power to second motor
        if (shooterMotorSecond != null) {
            double secondMotorPower = secondMotorReversed ? -actualPower : actualPower;
            shooterMotorSecond.setPower(secondMotorPower);
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

        if (shooterMotorFirst != null) {
            shooterMotorFirst.setPower(0);
        }
        if (shooterMotorSecond != null) {
            shooterMotorSecond.setPower(0);
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
        if (shooterMotorFirst == null) return;

        // Get current RPM from hardware velocity (much more accurate)
        double ticksPerSecond = shooterMotorFirst.getVelocity();
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
                double finalPower = reversed ? -power : power;

                // Apply power to both motors
                shooterMotorFirst.setPower(finalPower);
                if (shooterMotorSecond != null) {
                    double secondMotorPower = secondMotorReversed ? -finalPower : finalPower;
                    shooterMotorSecond.setPower(secondMotorPower);
                }

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
            shooterMotorFirst.setPower(0);
            if (shooterMotorSecond != null) {
                shooterMotorSecond.setPower(0);
            }
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
     * Get current power being applied to first motor
     */
    public double getCurrentPower() {
        if (shooterMotorFirst == null) return 0.0;
        return shooterMotorFirst.getPower();
    }

    public double getSecondMotorPower() {
        if (shooterMotorSecond == null) return 0.0;
        return shooterMotorSecond.getPower();
    }

    /**
     * Get current encoder position
     */
    public int getEncoderPosition() {
        if (shooterMotorFirst == null) return 0;
        return shooterMotorFirst.getCurrentPosition();
    }

    /**
     * Reset encoder position
     */
    public void resetEncoder() {
        if (shooterMotorFirst != null) {
            shooterMotorFirst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotorFirst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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