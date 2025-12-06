package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.config.Config;
import java.util.List;

/**
 * Simplified Turret Alignment Controller
 * Simple, straightforward logic - just turn toward the target
 */
@Config
public class SimpleTurretAlignmentController {

    private final LinearOpMode opMode;
    private final TurretController turretController;
    private final Limelight3A limelight;

    // Tunable parameters via FTC Dashboard
    public static double TOLERANCE_DEGREES = 2.0;
    public static double TURN_POWER = 0.8;
    public static int TARGET_TAG_ID = 20;

    // State
    private boolean isRunning = false;
    private double lastTx = 0;

    public SimpleTurretAlignmentController(LinearOpMode opMode, TurretController turretController) throws Exception {
        this.opMode = opMode;
        this.turretController = turretController;

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.pipelineSwitch(1);
            this.limelight.start();
        } catch (Exception e) {
            throw new Exception("Failed to initialize Limelight: " + e.getMessage());
        }
    }

    /**
     * Start alignment
     */
    public void start() {
        isRunning = true;
        opMode.telemetry.addLine("🎯 Alignment Started");
        opMode.telemetry.update();
    }

    /**
     * Stop alignment
     */
    public void stop() {
        isRunning = false;
        turretController.stop();
        opMode.telemetry.addLine("⏹ Alignment Stopped");
        opMode.telemetry.update();
    }

    /**
     * Update - call this every loop
     * Simple logic: read limelight, turn toward target if found
     */
    public void update() {
        if (!isRunning) {
            return;
        }

        // Try to find the AprilTag
        Double tx = getTargetTx();

        if (tx == null) {
            // No target found
            turretController.stop();
            opMode.telemetry.addLine("❌ No target found");
            opMode.telemetry.addData("Last TX", "%.2f°", lastTx);
            return;
        }

        lastTx = tx;

        // Check if we're aligned
        if (Math.abs(tx) <= TOLERANCE_DEGREES) {
            // Within tolerance - stop
            turretController.stop();
            opMode.telemetry.addLine("✅ ALIGNED!");
            opMode.telemetry.addData("TX Error", "%.2f°", tx);
            return;
        }

        // Not aligned - turn toward target
        // TX positive = target is RIGHT, so turn LEFT (negative power)
        // TX negative = target is LEFT, so turn RIGHT (positive power)
        double power = (tx > 0) ? -TURN_POWER : TURN_POWER;
        turretController.setPower(power);

        opMode.telemetry.addLine("🔄 Aligning...");
        opMode.telemetry.addData("TX Error", "%.2f°", tx);
        opMode.telemetry.addData("Turret Power", "%.2f", power);
    }

    /**
     * Get TX value from limelight for the target AprilTag
     * Returns null if target not found
     * EXACT COPY from TurretAlignmentController.findTarget()
     */
    private Double getTargetTx() {
        if (limelight == null) {
            return null;
        }

        try {
            int validReadings = 0;
            double tx = 0;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        tx = fiducial.getTargetXDegrees();
                        validReadings++;
                        break;
                    }
                }
            }

            if (validReadings > 0) {
                return -tx;  // Invert TX sign (same as LimelightAlignmentController)
            }
        } catch (Exception ignored) {
        }

        return null;
    }

    /**
     * Check if currently running
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Get the last TX value
     */
    public double getLastTx() {
        return lastTx;
    }
}
