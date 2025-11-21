package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controller for shooter motor (trigger-based variable speed)
 */
public class NewShooterController {

    private DcMotor shooterMotor;

    public double maxPower = 1.0;
    public boolean reversed = false;

    public NewShooterController(DcMotor motor) {
        this.shooterMotor = motor;
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Set shooter power based on trigger input (0.0 to 1.0)
     */
    public void setPower(double triggerValue) {
        if (shooterMotor != null) {
            double actualPower = triggerValue * maxPower;
            if (reversed) actualPower = -actualPower;
            shooterMotor.setPower(actualPower);
        }
    }

    /**
     * Get current power being applied
     */
    public double getCurrentPower() {
        if (shooterMotor == null) return 0.0;
        return shooterMotor.getPower();
    }
}