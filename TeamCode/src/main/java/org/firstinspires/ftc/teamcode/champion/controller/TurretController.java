package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controller for turret with encoder feedback
 */
public class TurretController {

    private CRServo turretServo;
    private AnalogInput turretEncoder;

    // Tracking variables
    private double lastPosition = 0;
    private double currentPosition = 0;
    private double velocity = 0;
    private double lastTime = 0;
    private ElapsedTime runtime;

    // Position control
    private boolean positionMode = false;
    private double targetPosition = 0;

    // Continuous rotation mode
    private enum RotationMode {
        NONE,
        CLOCKWISE,
        COUNTERCLOCKWISE
    }
    private RotationMode rotationMode = RotationMode.NONE;

    // Constants
    private static final double VOLTAGE_TO_DEGREES = 360.0 / 3.3;
    private static final double GEAR_RATIO = 2.5;

    // Tunable parameters
    public double servoPower = 0.5;
    public double positionTolerance = 2.0;
    public double manualSpeed = 0.6;
    public double joystickDeadband = 0.1;

    // Limits
    public double minAngle = 0.0;
    public double maxAngle = 144.0;
    public boolean enableLimits = false;

    // Presets
    public double frontPosition = 0.0;
    public double leftPosition = 36.0;
    public double backPosition = 72.0;
    public double rightPosition = 108.0;

    public TurretController(CRServo servo, AnalogInput encoder, ElapsedTime time) {
        this.turretServo = servo;
        this.turretEncoder = encoder;
        this.runtime = time;
        this.lastTime = time.seconds();
    }

    /**
     * Update position from encoder
     */
    public void update() {
        if (turretEncoder == null) return;

        double voltage = turretEncoder.getVoltage();
        double servoPosition = voltage * VOLTAGE_TO_DEGREES;
        currentPosition = servoPosition / GEAR_RATIO;

        // Calculate velocity
        double currentTime = runtime.seconds();
        double deltaTime = currentTime - lastTime;

        if (deltaTime > 0) {
            double deltaPosition = currentPosition - lastPosition;

            // Handle wraparound
            if (deltaPosition > 180) deltaPosition -= 360;
            else if (deltaPosition < -180) deltaPosition += 360;

            velocity = deltaPosition / deltaTime;
        }

        lastPosition = currentPosition;
        lastTime = currentTime;
    }

    /**
     * Set target position for automatic control
     */
    public void setTargetPosition(double target) {
        targetPosition = target;
        positionMode = true;
        rotationMode = RotationMode.NONE; // Cancel rotation mode
    }

    /**
     * Cancel automatic positioning
     */
    public void cancelPositionMode() {
        positionMode = false;
        rotationMode = RotationMode.NONE;
    }

    /**
     * Set continuous clockwise rotation
     */
    public void setClockwiseRotation() {
        positionMode = false;
        rotationMode = RotationMode.CLOCKWISE;
    }

    /**
     * Set continuous counterclockwise rotation
     */
    public void setCounterclockwiseRotation() {
        positionMode = false;
        rotationMode = RotationMode.COUNTERCLOCKWISE;
    }

    /**
     * Stop continuous rotation
     */
    public void stopRotation() {
        rotationMode = RotationMode.NONE;
    }

    /**
     * Calculate power based on manual input, rotation mode, or position mode
     */
    public double calculatePower(double manualInput) {
        double power = 0;

        // Manual control has highest priority
        if (Math.abs(manualInput) > joystickDeadband) {
            positionMode = false;
            rotationMode = RotationMode.NONE;
            power = manualInput * manualSpeed;

            // Apply soft limits
            if (enableLimits) {
                if (currentPosition <= minAngle && power < 0) power = 0;
                else if (currentPosition >= maxAngle && power > 0) power = 0;
            }
        } else if (rotationMode != RotationMode.NONE) {
            // Continuous rotation mode
            switch (rotationMode) {
                case CLOCKWISE:
                    power = servoPower;
                    break;
                case COUNTERCLOCKWISE:
                    power = -servoPower;
                    break;
            }

            // Apply soft limits
            if (enableLimits) {
                if (currentPosition <= minAngle && power < 0) {
                    power = 0;
                    rotationMode = RotationMode.NONE;
                } else if (currentPosition >= maxAngle && power > 0) {
                    power = 0;
                    rotationMode = RotationMode.NONE;
                }
            }
        } else if (positionMode) {
            // Position control mode
            double error = targetPosition - currentPosition;

            // Normalize error
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Check if reached target
            if (Math.abs(error) < positionTolerance) {
                power = 0;
                positionMode = false;
            } else {
                power = (error > 0) ? servoPower : -servoPower;
            }
        }

        return power;
    }

    /**
     * Set the turret power
     */
    public void setPower(double power) {
        if (turretServo != null) {
            turretServo.setPower(power);
        }
    }

    /**
     * Check if near a target position
     */
    public boolean isNear(double target) {
        double error = Math.abs(target - currentPosition);
        if (error > 180) error = 360 - error;
        return error < positionTolerance * 2;
    }

    // Getters
    public double getCurrentPosition() { return currentPosition; }
    public double getVelocity() { return velocity; }
    public boolean isPositionMode() { return positionMode; }
    public boolean isRotating() { return rotationMode != RotationMode.NONE; }
    public String getRotationMode() {
        switch (rotationMode) {
            case CLOCKWISE: return "CLOCKWISE";
            case COUNTERCLOCKWISE: return "COUNTERCLOCKWISE";
            default: return "NONE";
        }
    }
    public double getTargetPosition() { return targetPosition; }
    public double getRawVoltage() {
        return turretEncoder != null ? turretEncoder.getVoltage() : 0;
    }
    public double getServoPosition() {
        return getRawVoltage() * VOLTAGE_TO_DEGREES;
    }
}