package org.firstinspires.ftc.teamcode.champion.controller;

import android.annotation.SuppressLint;

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
    public static double MAX_TURN_POWER = 0.35;      // Maximum power (was TURN_POWER)
    public static double MIN_TURN_POWER = 0.3;       // Minimum power that still moves turret
    public static double PROPORTIONAL_GAIN = 0.015;  // Multiplier for error (power = error * gain)
    public static boolean USE_PROPORTIONAL = false;  // If true, use P control; if false, use linear interpolation
    public static double SLOWDOWN_THRESHOLD = 20.0;  // Start slowing down within this many degrees
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
    private String debugRawData = "";

    public SimpleTurretAlignmentController(LinearOpMode opMode, TurretController turretController) throws Exception {
        this.opMode = opMode;
        this.turretController = turretController;

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

            // Start the Limelight first
            this.limelight.start();

            // Wait for Limelight to initialize before switching pipeline
            try {
                Thread.sleep(500); // Give Limelight time to start
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            // Now switch to pipeline 1 (AprilTag detection)
            this.limelight.pipelineSwitch(1);

            // Wait for pipeline switch to complete
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            // Verify the pipeline switched
            try {
                LLStatus status = this.limelight.getStatus();
                opMode.telemetry.addData("Limelight Init", "Pipeline:%d FPS:%.0f",
                    status.getPipelineIndex(), status.getFps());
            } catch (Exception e) {
                opMode.telemetry.addData("Limelight Init", "Status check failed");
            }

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
            if (!debugRawData.isEmpty()) {
                opMode.telemetry.addData("Raw Debug", debugRawData);
            }
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

        // Not aligned - turn toward target with proportional control
        // Calculate power based on error magnitude
        double absTx = Math.abs(tx);
        double power;

        if (USE_PROPORTIONAL) {
            // True proportional control: power = error * gain
            power = absTx * PROPORTIONAL_GAIN;

            // Clamp to min/max power range
            power = Math.max(MIN_TURN_POWER, Math.min(MAX_TURN_POWER, power));
        } else {
            // Linear interpolation (original method)
            if (absTx > SLOWDOWN_THRESHOLD) {
                // Large error - use maximum power
                power = MAX_TURN_POWER;
            } else {
                // Within slowdown range - linear interpolation from MIN to MAX
                double ratio = absTx / SLOWDOWN_THRESHOLD;
                power = MIN_TURN_POWER + (MAX_TURN_POWER - MIN_TURN_POWER) * ratio;
            }
        }

        // Apply direction: TX positive = target is RIGHT, so turn LEFT (negative power)
        power = (tx > 0) ? -power : power;
        turretController.setPower(power);

        opMode.telemetry.addLine("🔄 Aligning...");
        opMode.telemetry.addData("TX Error", "%.2f°", tx);
        opMode.telemetry.addData("Turret Power", "%.2f", power);
        opMode.telemetry.addData("Error Magnitude", "%.1f°", absTx);
        opMode.telemetry.addData("Control Mode", USE_PROPORTIONAL ? "P-Control" : "Linear");
    }

    /**
     * Get TX value from limelight for the target AprilTag
     * Returns null if target not found
     * Takes multiple readings for better accuracy (same as LimelightAlignmentController)
     */
    @SuppressLint("DefaultLocale")
    private Double getTargetTx() {
        if (limelight == null) {
            return null;
        }

        try {
            // Get Limelight status first
            try {
                LLStatus status = limelight.getStatus();
                int currentPipeline = status.getPipelineIndex();
                double fps = status.getFps();

                limelightStatus = String.format("Pipeline:%d FPS:%.0f", currentPipeline, fps);

                // Warn if pipeline is wrong
                if (currentPipeline != 1) {
                    limelightStatus += " ⚠️WRONG!";
                    // Try to switch back to pipeline 1
                    limelight.pipelineSwitch(1);
                }

                // Warn if FPS is 0
                if (fps == 0) {
                    limelightStatus += " ❌NO STREAM";
                }
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

                // Debug: Log raw result data on first iteration
                if (i == 0) {
                    if (result == null) {
                        debugRawData = "LLResult=null";
                    } else if (!result.isValid()) {
                        debugRawData = String.format("Invalid result (tx=%.2f ty=%.2f)",
                            result.getTx(), result.getTy());
                    } else {
                        List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
                        debugRawData = String.format("Valid, Fiducials=%d",
                            fids != null ? fids.size() : -1);
                    }
                }

                if (result != null && result.isValid()) {
                    anyResultValid = true;
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null) {
                        totalFiducials = Math.max(totalFiducials, fiducials.size());

                        // Log all visible tag IDs for debugging
                        if (i == 0 && !fiducials.isEmpty()) {
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
