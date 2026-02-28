package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewShooterController {

    private final DcMotorEx shooterMotor1;  // Has encoder
    private final DcMotorEx shooterMotor2;  // No encoder, follows motor1
    private final ElapsedTime runtime;

    // Motor direction control (tune these if motors spin opposite ways)
    public static boolean motor1Reversed = false;
    public static boolean motor2Reversed = true;  // Often mounted opposite

    // Target RPM control
    public static double TARGET_RPM = 4600;
    public static double RPM_INCREMENT = 50.0;
    public static double MIN_RPM = 0.0;
    public static double MAX_RPM = 6000.0;

    // PID gains (tunable via FTC Dashboard)
    public static double HIGH_kP = 5;
    public static double HIGH_kI = 0.5;
    public static double HIGH_kD = 0.02;
    public static double HIGH_kF = 1;

    // PID limits
    public static double MAX_INTEGRAL = 10000.0;  // Integral windup limit
    public static double maxPower = 1.0;

    // RPM tolerance for "at target" check
    public static double RPM_TOLERANCE = 100.0;

    // Encoder ticks per revolution (for your motor)
    public static double TICKS_PER_REVOLUTION = 28;

    // PID state variables
    private double integralSum = 0;
    private double lastError = 0;
    private double lastPidTime = 0;

    // RPM tracking (motor1 only - motor2 has no encoder)
    private double currentRPM = 0;

    // Shooter state
    private boolean isShootMode = false;
    private double currentTargetRPM;

    /**
     * Constructor for dual-motor shooter
     * Motor1 has encoder, motor2 follows with same power
     */
    public NewShooterController(DcMotor motor1, DcMotor motor2) {
        this.runtime = new ElapsedTime();

        // Initialize motor 1 (with encoder)
        this.shooterMotor1 = castToMotorEx(motor1);
        if (shooterMotor1 != null) {
            shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Initialize motor 2 (no encoder, just follows)
        this.shooterMotor2 = castToMotorEx(motor2);
        if (shooterMotor2 != null) {
            shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        lastPidTime = runtime.seconds();
        currentTargetRPM = TARGET_RPM;
    }

    /**
     * Constructor for single-motor shooter (backwards compatible)
     */
    public NewShooterController(DcMotor motor) {
        this(motor, null);
    }

    private DcMotorEx castToMotorEx(DcMotor motor) {
        if (motor == null) return null;
        return (DcMotorEx) motor;
    }

    private void setMotorPowers(double power) {
        if (shooterMotor1 != null) {
            shooterMotor1.setPower(motor1Reversed ? -power : power);
        }
        if (shooterMotor2 != null) {
            shooterMotor2.setPower(motor2Reversed ? -power : power);
        }
    }

    /**
     * Set shooter power directly (for manual control)
     */
    public void setPower(double power) {
        double clampedPower = Math.max(-maxPower, Math.min(maxPower, power));
        setMotorPowers(clampedPower);
    }

    /**
     * Activate shoot mode - motors will try to reach target RPM
     */
    public void startShooting() {
        isShootMode = true;
        integralSum = 0;
        lastError = 0;
        lastPidTime = runtime.seconds();
    }

    /**
     * Stop shoot mode - motors will stop
     */
    public void stopShooting() {
        isShootMode = false;
        integralSum = 0;
        lastError = 0;
        setMotorPowers(0);
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
     * Toggle direction for motor 1
     */
    public void toggleMotor1Direction() {
        motor1Reversed = !motor1Reversed;
    }

    /**
     * Toggle direction for motor 2
     */
    public void toggleMotor2Direction() {
        motor2Reversed = !motor2Reversed;
    }

    /**
     * Toggle both motors' directions (legacy compatibility)
     */
    public void toggleShootDirection() {
        motor1Reversed = !motor1Reversed;
        motor2Reversed = !motor2Reversed;
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
        // Reset integral if target changed significantly (prevents overshoot)
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
        // Get RPM from motor1 encoder only (motor2 has no encoder)
        if (shooterMotor1 != null) {
            double tps = shooterMotor1.getVelocity();
            currentRPM = Math.abs((tps / TICKS_PER_REVOLUTION) * 60.0);
        }

        if (isShootMode && currentTargetRPM > 0) {
            double now = runtime.seconds();
            double dt = now - lastPidTime;

            // Run PID at high frequency
            if (dt >= 0.005) {  // 200Hz max
                double kP = HIGH_kP;
                double kI = HIGH_kI;
                double kD = HIGH_kD;
                double kF = HIGH_kF;

                double error = currentTargetRPM - currentRPM;

                // Feedforward (base power to reach target)
                double ffTerm = kF * currentTargetRPM;

                // Proportional
                double pTerm = kP * error;

                // Integral with anti-windup
                integralSum += error * dt;
                integralSum = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integralSum));
                double iTerm = kI * integralSum;

                // Derivative
                double dTerm = kD * (error - lastError) / dt;

                // Total output: feedforward + PID
                double power = ffTerm + pTerm + iTerm + dTerm;
                power = Math.max(0.0, Math.min(maxPower, power));

                setMotorPowers(power);

                lastError = error;
                lastPidTime = now;
            }
        } else if (!isShootMode) {
            setMotorPowers(0);
        }
    }

    /**
     * Get current RPM (from motor1 encoder)
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
        if (shooterMotor1 == null) return 0.0;
        return shooterMotor1.getPower();
    }

    /**
     * Get motor 1 encoder position
     */
    public int getEncoderPosition() {
        if (shooterMotor1 == null) return 0;
        return shooterMotor1.getCurrentPosition();
    }

    /**
     * Reset encoder
     */
    public void resetEncoder() {
        if (shooterMotor1 != null) {
            shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        currentRPM = 0;
        integralSum = 0;
        lastError = 0;
    }

    /**
     * Check if motors are running
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