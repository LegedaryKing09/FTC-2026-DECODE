package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * Limelight Mount Angle Calibration Tool
 * HOW TO USE:
 * 1. Measure the actual distance from your robot to the AprilTag (use a tape measure)
 * 2. Enter this distance in ACTUAL_DISTANCE_INCHES via FTC Dashboard
 * 3. Place your robot at that exact distance
 * 4. Run this OpMode
 * 5. Observe the telemetry showing:
 *    - Calculated distance (using current mount angle)
 *    - Error (difference from actual)
 *    - Suggested mount angle correction
 * 6. Adjust LIMELIGHT_MOUNT_ANGLE_DEGREES in FTC Dashboard until error is near zero
 * 7. Repeat at multiple distances (30", 60", 90", 120") to verify accuracy
 * 8. Use the final mount angle value in your AutoShootController
 */
@Config
@TeleOp(name = "Limelight Calibration Tool", group = "Calibration")
public class LimelightCalibrationTool extends LinearOpMode {

    // ========== CALIBRATION PARAMETERS ==========
    // Set these to match your robot's physical measurements
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 14.8;  // Distance from lens center to floor
    public static double GOAL_HEIGHT_INCHES = 29.5;            // Distance from target center to floor
    public static double LIMELIGHT_MOUNT_ANGLE_DEGREES = 25.0; // START WITH YOUR BEST ESTIMATE

    // Set this to your actual measured distance using a tape measure
    public static double ACTUAL_DISTANCE_INCHES = 60.0;        // MEASURE THIS WITH TAPE MEASURE!

    // AprilTag configuration
    public static int TARGET_APRILTAG_ID = 20;

    // Calculation settings
    public static int NUM_SAMPLES = 20;                        // Number of readings to average

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Set up dual telemetry (phone and dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addLine("✓ Limelight initialized successfully");
        } catch (Exception e) {
            telemetry.addLine("❌ ERROR: Failed to initialize Limelight");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("  LIMELIGHT CALIBRATION TOOL");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Measure actual distance to AprilTag");
        telemetry.addLine("2. Set ACTUAL_DISTANCE_INCHES in Dashboard");
        telemetry.addLine("3. Place robot at that distance");
        telemetry.addLine("4. Press START and observe readings");
        telemetry.addLine("5. Adjust LIMELIGHT_MOUNT_ANGLE_DEGREES");
        telemetry.addLine("   until error is near zero");
        telemetry.addLine();
        telemetry.addData("Current Mount Angle", "%.2f°", LIMELIGHT_MOUNT_ANGLE_DEGREES);
        telemetry.addData("Target Distance", "%.1f inches", ACTUAL_DISTANCE_INCHES);
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get distance reading
            CalibrationResult result = getCalibrationData();

            // Display results
            displayCalibrationResults(result);

            sleep(100); // Update 10 times per second
        }
    }

    /**
     * Get calibration data by averaging multiple readings
     */
    private CalibrationResult getCalibrationData() {
        CalibrationResult result = new CalibrationResult();

        double sumTy = 0;
        int validReadings = 0;

        // Collect multiple samples
        for (int i = 0; i < NUM_SAMPLES; i++) {
            try {
                LLResult llResult = limelight.getLatestResult();

                if (llResult != null && llResult.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                            double ty = fiducial.getTargetYDegrees();
                            sumTy += ty;
                            validReadings++;
                            break;
                        }
                    }
                }

                sleep(10); // Small delay between samples
            } catch (Exception e) {
                // Skip bad readings
            }
        }

        if (validReadings > 0) {
            result.hasTarget = true;
            result.avgTy = sumTy / validReadings;
            result.validSamples = validReadings;

            // Calculate distance using current mount angle
            result.calculatedDistance = calculateDistance(result.avgTy);

            // Calculate error
            result.error = result.calculatedDistance - ACTUAL_DISTANCE_INCHES;
            result.errorPercent = (result.error / ACTUAL_DISTANCE_INCHES) * 100.0;

            // Suggest angle correction
            result.suggestedAngleCorrection = calculateAngleCorrection(
                    result.avgTy,
                    ACTUAL_DISTANCE_INCHES
            );
        } else {
            result.hasTarget = false;
        }

        return result;
    }

    /**
     * Calculate distance using current mount angle
     */
    private double calculateDistance(double ty) {
        double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        double distance = (GOAL_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) /
                Math.tan(angleToGoalRadians);

        return Math.abs(distance);
    }

    /**
     * Calculate what the mount angle should be to get the correct distance
     */
    private double calculateAngleCorrection(double ty, double actualDistance) {
        // Work backwards from the desired distance
        // actualDistance = (goalHeight - lensHeight) / tan(correctAngle + ty)
        // tan(correctAngle + ty) = (goalHeight - lensHeight) / actualDistance
        // correctAngle + ty = atan((goalHeight - lensHeight) / actualDistance)
        // correctAngle = atan((goalHeight - lensHeight) / actualDistance) - ty

        double heightDiff = GOAL_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES;
        double correctTotalAngleRadians = Math.atan(heightDiff / actualDistance);
        double correctTotalAngleDegrees = correctTotalAngleRadians * (180.0 / Math.PI);

        return correctTotalAngleDegrees - ty;
    }

    /**
     * Display calibration results on telemetry
     */
    private void displayCalibrationResults(CalibrationResult result) {
        telemetry.clear();
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("  LIMELIGHT CALIBRATION RESULTS");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();

        if (!result.hasTarget) {
            telemetry.addLine("❌ NO TARGET FOUND");
            telemetry.addLine();
            telemetry.addLine("Make sure:");
            telemetry.addLine("- AprilTag is visible to Limelight");
            telemetry.addLine("- Correct AprilTag ID is set");
            telemetry.addLine("- Limelight pipeline is configured");
            telemetry.addData("Looking for Tag ID", TARGET_APRILTAG_ID);
        } else {
            // Target found - show results
            telemetry.addLine("✓ TARGET LOCKED");
            telemetry.addData("Valid Samples", "%d / %d", result.validSamples, NUM_SAMPLES);
            telemetry.addLine();

            telemetry.addLine("━━━ MEASUREMENTS ━━━");
            telemetry.addData("Actual Distance", "%.1f inches", ACTUAL_DISTANCE_INCHES);
            telemetry.addData("Calculated Distance", "%.1f inches", result.calculatedDistance);
            telemetry.addLine();

            telemetry.addLine("━━━ ERROR ANALYSIS ━━━");
            telemetry.addData("Error", "%.2f inches", result.error);
            telemetry.addData("Error Percent", "%.1f%%", result.errorPercent);
            telemetry.addLine();

            // Color-coded accuracy feedback
            double absError = Math.abs(result.error);
            if (absError < 1.0) {
                telemetry.addLine("✓ EXCELLENT - Error < 1 inch");
            } else if (absError < 3.0) {
                telemetry.addLine("✓ GOOD - Error < 3 inches");
            } else if (absError < 6.0) {
                telemetry.addLine("⚠ FAIR - Adjust mount angle");
            } else {
                telemetry.addLine("❌ POOR - Adjust mount angle");
            }
            telemetry.addLine();

            telemetry.addLine("━━━ CURRENT SETTINGS ━━━");
            telemetry.addData("Lens Height", "%.1f in", LIMELIGHT_LENS_HEIGHT_INCHES);
            telemetry.addData("Goal Height", "%.1f in", GOAL_HEIGHT_INCHES);
            telemetry.addData("Mount Angle", "%.2f°", LIMELIGHT_MOUNT_ANGLE_DEGREES);
            telemetry.addData("Measured ty", "%.2f°", result.avgTy);
            telemetry.addLine();

            telemetry.addLine("━━━ SUGGESTED CORRECTION ━━━");
            telemetry.addData("Suggested Mount Angle", "%.2f°", result.suggestedAngleCorrection);
            double angleDiff = result.suggestedAngleCorrection - LIMELIGHT_MOUNT_ANGLE_DEGREES;
            telemetry.addData("Adjustment Needed", "%+.2f°", angleDiff);
            telemetry.addLine();

            telemetry.addLine("▶ Adjust LIMELIGHT_MOUNT_ANGLE_DEGREES");
            telemetry.addLine("  in FTC Dashboard to reduce error");
        }

        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.update();
    }

    /**
     * Data structure to hold calibration results
     */
    private static class CalibrationResult {
        boolean hasTarget = false;
        double avgTy = 0;
        int validSamples = 0;
        double calculatedDistance = 0;
        double error = 0;
        double errorPercent = 0;
        double suggestedAngleCorrection = 0;
    }
}