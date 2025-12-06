package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
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

    // Debug info
    private int lastValidReadings = 0;
    private int lastTotalFiducials = 0;
    private boolean lastResultValid = false;
    private String lastSeenTagIds = "";
    private String limelightStatus = "Unknown";

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
            opMode.telemetry.addData("Valid Readings", "%d/5", lastValidReadings);
            opMode.telemetry.addData("Total Fiducials", lastTotalFiducials);
            opMode.telemetry.addData("Result Valid", lastResultValid);
            opMode.telemetry.addData("Target Tag ID", TARGET_TAG_ID);
            opMode.telemetry.addData("Seen Tag IDs", lastSeenTagIds.isEmpty() ? "none" : lastSeenTagIds);
            opMode.telemetry.addData("Limelight Status", limelightStatus);
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
     * Takes multiple readings for better accuracy (same as LimelightAlignmentController)
     */
    private Double getTargetTx() {
        if (limelight == null) {
            return null;
        }

        try {
            // Get Limelight status first
            try {
                LLStatus status = limelight.getStatus();
                limelightStatus = String.format("Pipeline:%d FPS:%.0f",
                    status.getPipelineIndex(), status.getFps());
            } catch (Exception e) {
                limelightStatus = "Error: " + e.getMessage();
            }

            // Take multiple readings for better accuracy (same as LimelightAlignmentController)
            double sumTx = 0;
            int validReadings = 0;
            int totalFiducials = 0;
            boolean anyResultValid = false;
            StringBuilder seenIds = new StringBuilder();

            for (int i = 0; i < 5; i++) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    anyResultValid = true;
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null) {
                        totalFiducials = Math.max(totalFiducials, fiducials.size());

                        // Log all visible tag IDs for debugging
                        if (i == 0 && fiducials.size() > 0) {
                            for (LLResultTypes.FiducialResult fid : fiducials) {
                                if (seenIds.length() > 0) seenIds.append(", ");
                                seenIds.append(fid.getFiducialId());
                            }
                        }

                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                                sumTx += fiducial.getTargetXDegrees();
                                validReadings++;
                                break;
                            }
                        }
                    }
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return null;
                }
            }

            // Store debug info
            lastValidReadings = validReadings;
            lastTotalFiducials = totalFiducials;
            lastResultValid = anyResultValid;
            lastSeenTagIds = seenIds.toString();

            if (validReadings > 0) {
                double avgTx = sumTx / validReadings;
                return -avgTx;  // Invert TX sign (same as LimelightAlignmentController)
            }
        } catch (Exception e) {
            opMode.telemetry.addData("Exception", e.getMessage());
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
