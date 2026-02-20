package org.firstinspires.ftc.teamcode.champion.controller;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Controller for two uptake servos (reversed by default)
 */
public class UptakeController {

    private final CRServo uptakeServo1;
    private final CRServo uptakeServo2;
    private boolean isActive = false;

    public double power = -1.0;  // Reversed by default as requested
    public boolean reversed = false;

    public UptakeController(CRServo servo1, CRServo servo2) {
        this.uptakeServo1 = servo1;
        this.uptakeServo2 = servo2;
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
        double actualPower = isActive ? power : 0.0;
        if (reversed) actualPower = -actualPower;

        if (uptakeServo1 != null) {
            uptakeServo1.setPower(actualPower);
        }
        if (uptakeServo2 != null) {
            uptakeServo2.setPower(actualPower);
        }
    }

    public boolean isActive() { return isActive; }
}