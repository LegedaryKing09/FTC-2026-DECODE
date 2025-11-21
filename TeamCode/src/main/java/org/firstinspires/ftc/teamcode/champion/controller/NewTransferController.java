package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controller for transfer motor
 */
public class NewTransferController {

    private DcMotor transferMotor;
    private boolean isActive = false;

    public double power = 1.0;
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

    /**
     * Set transfer state
     */
    public void setState(boolean active) {
        isActive = active;
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

    public boolean isActive() { return isActive; }
}