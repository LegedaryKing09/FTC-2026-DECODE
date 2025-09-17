package org.firstinspires.ftc.teamcode.champion.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.List;
import java.util.ArrayList;

/**
 * Limelight Distance Calibration for DECODE FTC
 *
 * DECODE AprilTag Specifications:
 * - Size: 8.125 inches square (not 6.75!)
 * - Tag Family: 36h11
 * - Tag Center Height: 29.5 inches from ground
 * - Placed on front face of GOAL
 *
 * This program helps find the optimal camera mounting angle
 * for accurate distance measurement.
 */
@Config
@TeleOp(name = "Limelight Distance Calibration", group = "Calibration")
public class LimelightApriltagDistance extends LinearOpMode {

    private Limelight3A limelight;
    private FtcDashboard dashboard;

    // DECODE OFFICIAL SPECIFICATIONS
    private static final double GOAL_TAG_HEIGHT = 29.5;  // Tag center height in inches (DECODE spec)
    private static final double TAG_SIZE = 6.75;        // Tag size in inches (DECODE spec)

    // TUNABLE PARAMETERS (Adjust these via FTC Dashboard)
    @Config
    public static class CameraConfig {
        // Camera mounting parameters - ADJUST THESE
        public static double CAMERA_HEIGHT = 14.8;       // Height of camera lens from ground
        public static double CAMERA_ANGLE = 9.57;        // Mounting angle (0=horizontal, positive=tilted up)

        // Test parameters
        public static double TEST_DISTANCE = 49.2;       // Known test distance for calibration
        public static boolean USE_RADIANS = false;       // Toggle for angle display
    }

    // Data storage for calibration
    private ArrayList<CalibrationData> calibrationHistory = new ArrayList<>();
    private double currentCalculatedDistance = 0;
    private double currentTY = 0;
    private boolean hasTarget = false;

    private class CalibrationData {
        double actualDistance;
        double measuredDistance;
        double ty;
        double cameraAngle;
        double error;
        long timestamp;

        CalibrationData(double actual, double measured, double ty, double angle) {
            this.actualDistance = actual;
            this.measuredDistance = measured;
            this.ty = ty;
            this.cameraAngle = angle;
            this.error = measured - actual;
            this.timestamp = System.currentTimeMillis();
        }
    }

    @Override
    public void runOpMode() {

        // Initialize
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // AprilTag pipeline
            limelight.start();
            telemetry.addLine("✓ Limelight Connected");
        } catch (Exception e) {
            telemetry.addLine("✗ Limelight Error: " + e.getMessage());
            telemetry.update();
            return;
        }

        displayInstructions();
        waitForStart();

        if (isStopRequested()) return;

        // Main calibration loop
        while (opModeIsActive()) {

            // Update measurements
            updateMeasurements();

            // Display current data
            displayCalibrationData();

            // Handle controls
            handleControls();

            // Send dashboard data
            sendDashboardData();

            sleep(50); // 20Hz update
        }

        limelight.stop();
    }

    /**
     * Update distance measurements using official Limelight formula
     */
    private void updateMeasurements() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get ty from result (overall target)
            currentTY = result.getTy();

            // Also check for specific AprilTags
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            hasTarget = !fiducials.isEmpty();

            if (hasTarget) {
                // Use first detected tag for more precise ty
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                currentTY = fiducial.getTargetYDegrees();

                // Calculate distance using official Limelight formula
                currentCalculatedDistance = calculateDistance(currentTY);
            }
        } else {
            hasTarget = false;
        }
    }

    /**
     * Calculate distance using official Limelight formula
     * d = (h2 - h1) / tan(a1 + a2)
     *
     * Where:
     * h2 = target height (29.5" for DECODE)
     * h1 = camera height
     * a1 = camera mounting angle
     * a2 = ty from Limelight
     */
    private double calculateDistance(double ty) {
        // Official Limelight formula
        double angleToGoalDegrees = CameraConfig.CAMERA_ANGLE + ty;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        // Calculate horizontal distance to tag
        double distanceInches = (GOAL_TAG_HEIGHT - CameraConfig.CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);

        // Return absolute value (distance is always positive)
        return Math.abs(distanceInches);
    }

    /**
     * Calculate optimal camera angle for current setup
     * (Reverse calculation when at known distance)
     */
    private double calculateOptimalAngle(double knownDistance, double ty) {
        // Rearrange formula to solve for a1:
        // tan(a1 + a2) = (h2 - h1) / d
        // a1 + a2 = atan((h2 - h1) / d)
        // a1 = atan((h2 - h1) / d) - a2

        double heightDiff = GOAL_TAG_HEIGHT - CameraConfig.CAMERA_HEIGHT;
        double angleSum = Math.atan(heightDiff / knownDistance);
        double angleSumDegrees = angleSum * (180.0 / Math.PI);

        return angleSumDegrees - ty;
    }

    /**
     * Display instructions
     */
    private void displayInstructions() {
        telemetry.clear();
        telemetry.addLine("═══ LIMELIGHT DISTANCE CALIBRATION ═══");
        telemetry.addLine();
        telemetry.addLine("DECODE SPECIFICATIONS:");
        telemetry.addLine("• Tag Height: 29.5 inches");
        telemetry.addLine("• Tag Size: 8.125 inches");
        telemetry.addLine();
        telemetry.addLine("CALIBRATION PROCEDURE:");
        telemetry.addLine("1. Place tag at EXACTLY 24 inches");
        telemetry.addLine("2. Adjust CAMERA_ANGLE until distance reads 24\"");
        telemetry.addLine("3. Test at 12\", 36\", 48\" to verify");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("A = Save calibration point");
        telemetry.addLine("B = Calculate optimal angle");
        telemetry.addLine("X = Clear history");
        telemetry.addLine("Y = Export results");
        telemetry.addLine();
        telemetry.addLine(">>> Press START to begin <<<");
        telemetry.update();
    }

    /**
     * Display current calibration data
     */
    private void displayCalibrationData() {
        telemetry.clear();
        telemetry.addLine("═══ DISTANCE MEASUREMENT ═══");
        telemetry.addLine();

        if (hasTarget) {
            telemetry.addLine("✓ TAG DETECTED");
            telemetry.addLine();

            // Current measurements
            telemetry.addLine("CURRENT MEASUREMENTS:");
            telemetry.addData("TY Angle", "%.2f°", currentTY);
            telemetry.addData("Camera Height", "%.1f inches", CameraConfig.CAMERA_HEIGHT);
            telemetry.addData("Camera Angle", "%.1f°", CameraConfig.CAMERA_ANGLE);
            telemetry.addLine();

            // Calculated distance
            telemetry.addLine("CALCULATED DISTANCE:");
            telemetry.addData("Distance", "%.2f inches", currentCalculatedDistance);
            telemetry.addLine();

            // Optimal angle calculation
            double optimalAngle = calculateOptimalAngle(CameraConfig.TEST_DISTANCE, currentTY);
            telemetry.addLine("FOR TEST DISTANCE " + CameraConfig.TEST_DISTANCE + " inches:");
            telemetry.addData("Optimal Camera Angle", "%.2f°", optimalAngle);
            telemetry.addData("Current Error", "%.2f inches",
                    currentCalculatedDistance - CameraConfig.TEST_DISTANCE);

        } else {
            telemetry.addLine("✗ NO TAG DETECTED");
            telemetry.addLine("Point camera at DECODE AprilTag");
        }

        telemetry.addLine();
        telemetry.addLine("Adjust CAMERA_ANGLE in Dashboard");
        telemetry.addLine("Press A to save measurement");

        // Show recent calibration points
        if (!calibrationHistory.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine("CALIBRATION HISTORY:");
            int start = Math.max(0, calibrationHistory.size() - 3);
            for (int i = start; i < calibrationHistory.size(); i++) {
                CalibrationData data = calibrationHistory.get(i);
                telemetry.addData(String.format("%d: %.0f\"", i+1, data.actualDistance),
                        "Measured: %.1f\" Error: %.1f\"",
                        data.measuredDistance, data.error);
            }
        }

        telemetry.update();
    }

    /**
     * Handle gamepad controls
     */
    private void handleControls() {
        // A - Save calibration point
        if (gamepad1.a && hasTarget) {
            CalibrationData data = new CalibrationData(
                    CameraConfig.TEST_DISTANCE,
                    currentCalculatedDistance,
                    currentTY,
                    CameraConfig.CAMERA_ANGLE
            );
            calibrationHistory.add(data);

            // Brief feedback
            telemetry.addLine("✓ Saved measurement");
            telemetry.update();
            sleep(500);
        }

        // B - Calculate and display optimal angle
        if (gamepad1.b && hasTarget) {
            calculateAndDisplayOptimalSettings();
        }

        // X - Clear history
        if (gamepad1.x) {
            calibrationHistory.clear();
            telemetry.addLine("History cleared");
            telemetry.update();
            sleep(500);
        }

        // Y - Export/display all results
        if (gamepad1.y) {
            displayFullResults();
        }
    }

    /**
     * Calculate and display optimal settings
     */
    private void calculateAndDisplayOptimalSettings() {
        telemetry.clear();
        telemetry.addLine("═══ OPTIMAL SETTINGS ═══");
        telemetry.addLine();

        double optimalAngle = calculateOptimalAngle(CameraConfig.TEST_DISTANCE, currentTY);

        telemetry.addLine("Place camera at known distance: " + CameraConfig.TEST_DISTANCE + " inches");
        telemetry.addLine();
        telemetry.addData("Set Camera Height", "%.1f inches", CameraConfig.CAMERA_HEIGHT);
        telemetry.addData("Set Camera Angle", "%.2f degrees", optimalAngle);
        telemetry.addLine();
        telemetry.addLine("This angle will give accurate distance");
        telemetry.addLine("measurements for DECODE AprilTags");
        telemetry.addLine();

        // Test at different distances
        telemetry.addLine("EXPECTED READINGS:");
        double[] testDistances = {12, 24, 36, 48};
        for (double d : testDistances) {
            double expectedTY = calculateExpectedTY(d, optimalAngle);
            telemetry.addData(String.format("At %.0f\"", d), "TY should be %.1f°", expectedTY);
        }

        telemetry.addLine();
        telemetry.addLine("Press BACK to continue");
        telemetry.update();

        while (!gamepad1.back && opModeIsActive()) {
            sleep(50);
        }
    }

    /**
     * Calculate expected TY for a given distance and camera angle
     */
    private double calculateExpectedTY(double distance, double cameraAngle) {
        double heightDiff = GOAL_TAG_HEIGHT - CameraConfig.CAMERA_HEIGHT;
        double angleRadians = Math.atan(heightDiff / distance);
        double angleDegrees = angleRadians * (180.0 / Math.PI);
        return angleDegrees - cameraAngle;
    }

    /**
     * Display full calibration results
     */
    private void displayFullResults() {
        telemetry.clear();
        telemetry.addLine("═══ CALIBRATION RESULTS ═══");
        telemetry.addLine();

        if (calibrationHistory.isEmpty()) {
            telemetry.addLine("No calibration data yet");
        } else {
            // Calculate statistics
            double sumError = 0;
            double sumAbsError = 0;
            for (CalibrationData data : calibrationHistory) {
                sumError += data.error;
                sumAbsError += Math.abs(data.error);
            }

            double avgError = sumError / calibrationHistory.size();
            double avgAbsError = sumAbsError / calibrationHistory.size();

            telemetry.addData("Measurements", calibrationHistory.size());
            telemetry.addData("Avg Error", "%.2f inches", avgError);
            telemetry.addData("Avg Abs Error", "%.2f inches", avgAbsError);
            telemetry.addLine();

            // Accuracy assessment
            if (avgAbsError < 0.5) {
                telemetry.addLine("✓ EXCELLENT - Ready for competition!");
            } else if (avgAbsError < 1.0) {
                telemetry.addLine("✓ GOOD - Acceptable accuracy");
            } else if (avgAbsError < 2.0) {
                telemetry.addLine("⚠ FAIR - Needs adjustment");
            } else {
                telemetry.addLine("✗ POOR - Recalibrate needed");
            }
        }

        telemetry.addLine();
        telemetry.addLine("Press BACK to continue");
        telemetry.update();

        while (!gamepad1.back && opModeIsActive()) {
            sleep(50);
        }
    }

    /**
     * Send data to FTC Dashboard
     */
    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();

        // Current measurements
        packet.put("Has_Target", hasTarget);
        packet.put("TY_Angle", currentTY);
        packet.put("Calculated_Distance", currentCalculatedDistance);

        // Configuration
        packet.put("Camera_Height", CameraConfig.CAMERA_HEIGHT);
        packet.put("Camera_Angle", CameraConfig.CAMERA_ANGLE);
        packet.put("Test_Distance", CameraConfig.TEST_DISTANCE);

        // Optimal angle
        if (hasTarget) {
            double optimalAngle = calculateOptimalAngle(CameraConfig.TEST_DISTANCE, currentTY);
            packet.put("Optimal_Angle", optimalAngle);
            packet.put("Distance_Error", currentCalculatedDistance - CameraConfig.TEST_DISTANCE);
        }

        // Graph distance over time
        packet.put("Distance_Graph", currentCalculatedDistance);

        dashboard.sendTelemetryPacket(packet);
    }
}