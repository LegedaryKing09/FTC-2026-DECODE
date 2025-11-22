package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controller for shooter motor with target RPM control
 */
@Config
public class NewShooterController {

    private DcMotor shooterMotor;
    private ElapsedTime runtime;

    // Configurable parameters
    public static double maxPower = 1.0;
    public static boolean reversed = false;

    // Target RPM control
    public static double TARGET_RPM = 1500;  // Default target RPM
    public static double RPM_INCREMENT = 100.0; // How much to increase/decrease per button press
    public static double MIN_RPM = 0.0;
    public static double MAX_RPM = 6000.0;

    // RPM control tuning
    public static double RPM_kP = 0.0003;  // Proportional gain for RPM control
    public static double RPM_TOLERANCE = 10.0; // RPM tolerance to consider "at target"

    // Encoder ticks per revolution
    public static double TICKS_PER_REVOLUTION = 28;

    // RPM calculation variables
    private int lastEncoderPosition = 0;
    private double lastTime = 0;
    private double currentRPM = 0;
    private static final double RPM_UPDATE_INTERVAL = 0.1; // Update every 100ms

    // Shooter state
    private boolean isShootMode = false; // Whether we're actively trying to reach target RPM
    private double currentTargetRPM = TARGET_RPM;

    public NewShooterController(DcMotor motor) {
        this.shooterMotor = motor;
        this.runtime = new ElapsedTime();

        if (motor != null) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lastEncoderPosition = motor.getCurrentPosition();
            lastTime = runtime.seconds();
        }

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
    }

    /**
     * Stop shoot mode - motor will stop
     */
    public void stopShooting() {
        isShootMode = false;
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
     * Update RPM calculation and control - call this every loop
     */
    public void update() {
        if (shooterMotor == null) return;

        // Update RPM measurement
        updateRPM();

        // If in shoot mode, control motor to reach target RPM
        if (isShootMode) {
            double error = currentTargetRPM - currentRPM;

            // Simple P control for RPM
            double power = error * RPM_kP;

            // Clamp power
            power = Math.max(-maxPower, Math.min(maxPower, power));

            // Apply direction
            if (reversed) power = -power;

            shooterMotor.setPower(power);
        }
    }

    /**
     * Internal method to update RPM from encoder
     */
    private void updateRPM() {
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;

        // Update RPM at regular intervals
        if (deltaTime >= RPM_UPDATE_INTERVAL) {
            int currentPosition = shooterMotor.getCurrentPosition();
            int deltaTicks = currentPosition - lastEncoderPosition;

            // Calculate RPM
            if (deltaTime > 0) {
                double ticksPerSecond = deltaTicks / deltaTime;
                double revolutionsPerSecond = ticksPerSecond / TICKS_PER_REVOLUTION;
                currentRPM = Math.abs(revolutionsPerSecond * 60.0);
            }

            lastEncoderPosition = currentPosition;
            lastTime = currentTime;
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
            lastEncoderPosition = 0;
            currentRPM = 0;
        }
    }

    /**
     * Check if motor is running
     */
    public boolean isRunning() {
        return getCurrentPower() != 0;
    }
}