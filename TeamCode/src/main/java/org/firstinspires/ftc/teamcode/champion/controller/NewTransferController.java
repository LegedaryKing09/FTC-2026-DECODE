package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controller for transfer motor
 */
@Config
public class NewTransferController {

    private final DcMotor transferMotor;
    private boolean isActive = false;

    // Configurable via FTC Dashboard
    public static double power = -1.0;
    public boolean reversed = false;

    public NewTransferController(DcMotor motor) {
        this.transferMotor = motor;
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Toggle transfer on/off
     */
    public void toggle() {
        isActive = !isActive;
    }

    public void toggleDirection() {
        reversed = !reversed;
    }

    /**
     * Set transfer state
     */
    public void setState(boolean active) {
        isActive = active;
    }

    /**
     * Turn transfer on
     */
    public void start() {
        isActive = true;
    }

    /**
     * Turn transfer off
     */
    public void stop() {
        isActive = false;
    }

    /**
     * Update motor power based on state
     */
    public void update() {
        if (transferMotor != null) {
            double actualPower = isActive ? power : 0.0;
            if (reversed) actualPower = -actualPower;
            transferMotor.setPower(actualPower);
        }
    }

    /**
     * Get current power being applied
     */
    public double getCurrentPower() {
        if (transferMotor == null) return 0.0;
        return transferMotor.getPower();
    }

    /**
     * Check if transfer is active
     */
    public boolean isActive() {
        return isActive;
    }
}