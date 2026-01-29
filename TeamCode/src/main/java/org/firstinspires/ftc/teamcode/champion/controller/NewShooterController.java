package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewShooterController {

    private final DcMotorEx shooterMotor;
    private final ElapsedTime runtime;

    // Motor direction control
    public static boolean motorReversed = false;

    // Target RPM control
    public static double TARGET_RPM = 4600;
    public static double RPM_INCREMENT = 50.0;
    public static double MIN_RPM = 0.0;
    public static double MAX_RPM = 6000.0;

    // PID gains (tunable via FTC Dashboard)
    public static double kP = 0.65;
    public static double kI = 0;
    public static double kD = 0.001;

    // Feedforward gain
    public static double kF = 0;

    // PID limits
    public static double MAX_INTEGRAL = 10000.0;
    public static double maxPower = 1.0;

    // RPM tolerance for "at target" check
    public static double RPM_TOLERANCE = 50.0;

    // Encoder ticks per revolution
    public static double TICKS_PER_REVOLUTION = 28;

    // PID state variables
    private double integralSum = 0;
    private double lastError = 0;
    private double lastPidTime = 0;

    // RPM tracking
    private double currentRPM = 0;

    // Shooter state
    private boolean isShootMode = false;
    private double currentTargetRPM;

    /**
     * Constructor for single-motor shooter
     */
    public NewShooterController(DcMotor motor) {
        this.runtime = new ElapsedTime();

        this.shooterMotor = (DcMotorEx) motor;
        if (shooterMotor != null) {
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        lastPidTime = runtime.seconds();
        currentTargetRPM = TARGET_RPM;
    }

    private void setMotorPower(double power) {
        if (shooterMotor != null) {
            shooterMotor.setPower(motorReversed ? -power : power);
        }
    }

    /**
     * Set shooter power directly (for manual control)
     */
    public void setPower(double power) {
        double clampedPower = Math.max(-maxPower, Math.min(maxPower, power));
        setMotorPower(clampedPower);
    }

    /**
     * Activate shoot mode - motor will try to reach target RPM
     */
    public void startShooting() {
        isShootMode = true;
        integralSum = 0;
        lastError = 0;
        lastPidTime = runtime.seconds();
    }

    /**
     * Stop shoot mode - motor will stop
     */
    public void stopShooting() {
        isShootMode = false;
        integralSum = 0;
        lastError = 0;
        setMotorPower(0);
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

    /**
     * Toggle motor direction
     */
    public void toggleShootDirection() {
        motorReversed = !motorReversed;
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
        double newTarget = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
        if (Math.abs(newTarget - currentTargetRPM) > 500) {
            integralSum = 0;
        }
        currentTargetRPM = newTarget;
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
        if (shooterMotor != null) {
            double tps = shooterMotor.getVelocity();
            currentRPM = Math.abs((tps / TICKS_PER_REVOLUTION) * 60.0);
        }

        if (isShootMode && currentTargetRPM > 0) {
            double now = runtime.seconds();
            double dt = now - lastPidTime;

            if (dt >= 0.005) {  // 200Hz max
                double error = currentTargetRPM - currentRPM;

                // Feedforward
                double ffTerm = kF * currentTargetRPM;

                // Proportional
                double pTerm = kP * error;

                // Integral with anti-windup
                integralSum += error * dt;
                integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
                double iTerm = kI * integralSum;

                // Derivative
                double dTerm = kD * (error - lastError) / dt;

                // Total output
                double power = ffTerm + pTerm + iTerm + dTerm;
                power = Math.max(0.0, Math.min(maxPower, power));

                setMotorPower(power);

                lastError = error;
                lastPidTime = now;
            }
        } else if (!isShootMode) {
            setMotorPower(0);
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
     * Get encoder position
     */
    public int getEncoderPosition() {
        if (shooterMotor == null) return 0;
        return shooterMotor.getCurrentPosition();
    }

    /**
     * Reset encoder
     */
    public void resetEncoder() {
        if (shooterMotor != null) {
            shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        currentRPM = 0;
        integralSum = 0;
        lastError = 0;
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