package org.firstinspires.ftc.teamcode.champion.controller;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Controller for uptake servo (reversed by default)
 */
public class UptakeController {

    private CRServo uptakeServo;
    private boolean isActive = false;

    public double power = -1.0;  // Reversed by default as requested

    public UptakeController(CRServo servo) {
        this.uptakeServo = servo;
    }

    /**
     * Toggle uptake on/off
     */
    public void toggle() {
        isActive = !isActive;
    }

    /**
     * Set uptake state
     */
    public void setState(boolean active) {
        isActive = active;
    }

    /**
     * Update servo power based on state
     */
    public void update() {
        if (uptakeServo != null) {
            double actualPower = isActive ? power : 0.0;
            uptakeServo.setPower(actualPower);
        }
    }

    /**
     * Get current power being applied
     */
    public double getCurrentPower() {
        if (uptakeServo == null || !isActive) return 0.0;
        return power;
    }

    public boolean isActive() { return isActive; }
}