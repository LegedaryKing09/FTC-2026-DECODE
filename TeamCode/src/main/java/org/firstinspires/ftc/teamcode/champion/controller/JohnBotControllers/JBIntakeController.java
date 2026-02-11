package org.firstinspires.ftc.teamcode.champion.controller.JohnBotControllers;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controller for intake motor
 */
public class JBIntakeController {

    private final DcMotor intakeMotor;
    private boolean isActive = false;

    public double power = -1.0;
    public boolean reversed = false;

    public JBIntakeController(DcMotor motor) {
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

    public void toggleDirection() {
        reversed = !reversed;
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

    /**
     * Get current power being applied
     */
    public double getCurrentPower() {
        if (intakeMotor == null) return 0.0;
        return intakeMotor.getPower();
    }

    public boolean isActive() { return isActive; }
}