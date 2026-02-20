package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controller for intake motor
 */
public class NewIntakeController {

    private final DcMotor intakeMotor;
    private boolean isActive = false;

    public double power = -1.0;
    public boolean reversed = false;

    public NewIntakeController(DcMotor motor) {
        this.intakeMotor = motor;
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Toggle intake on/off
     */
    public void toggle() {
        isActive = !isActive;
    }

    /**
     * Set intake state
     */
    public void setState(boolean active) {
        isActive = active;
    }

    /**
     * Update motor power based on state
     */
    public void update() {
        if (intakeMotor != null) {
            double actualPower = isActive ? power : 0.0;
            if (reversed) actualPower = -actualPower;
            intakeMotor.setPower(actualPower);
        }
    }

    public boolean isActive() { return isActive; }
}