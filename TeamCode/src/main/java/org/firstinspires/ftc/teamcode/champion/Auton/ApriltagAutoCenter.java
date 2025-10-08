package org.firstinspires.ftc.teamcode.champion.Auton;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

import java.util.List;

/**
 * AprilTag Centering Autonomous
 * - Detects AprilTag and rotates to center on it
 * - No forward/backward movement - rotation only
 * - Stops when centered within tolerance
 */
@Config
@Autonomous(name = "Center on AprilTag", group = "Champion")
public class ApriltagAutoCenter extends LinearOpMode {

    // Hardware
    private SixWheelDriveController driveController;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    // Constants
    private static final int TARGET_TAG_ID = 2;  // Track tag 2 (change as needed)

    // Tunable parameters via FTC Dashboard
    @Config
    public static class CenteringParams {
        // Control gains
        public static double KP_ROTATION = 0.015;          // Proportional gain for rotation
        public static double MIN_ROTATION_POWER = 0.08;    // Minimum power to overcome friction
        public static double MAX_ROTATION_POWER = 0.4;     // Maximum rotation speed

        // Tolerances
        public static double CENTERING_TOLERANCE = 1.0;    // Degrees - considered centered
        public static double STABLE_TIME_MS = 500;         // Time to hold position before declaring success

        // Deadband
        public static double ROTATION_DEADBAND = 0.5;      // Degrees - ignore small errors
    }

    // Tracking data
    private double tx = 0;                  // Horizontal angle to tag (degrees)
    private boolean hasTarget = false;
    private long centeredStartTime = 0;
    private boolean isCentered = false;
    private int totalFrames = 0;
    private int framesWithTarget = 0;

    @Override
    public void runOpMode() {

        // Initialize hardware
        initializeHardware();

        // Display startup info
        displayStartup();

        waitForStart();

        if (isStopRequested()) return;

        // Main control loop
        while (opModeIsActive() && !isCentered) {

            // Update tracking from Limelight
            updateTracking();

            if (hasTarget) {
                // We see the tag - rotate to center on it
                performCentering();

                // Check if we're centered and stable
                checkCenteringStatus();

                // Display status
                displayTrackingInfo();

                framesWithTarget++;
            } else {
                // No tag visible - stop and wait
                driveController.stopDrive();
                centeredStartTime = 0;  // Reset centering timer
                displayNoTarget();
            }

            // Update odometry
            driveController.updateOdometry();

            // Send data to dashboard
            sendDashboardData();

            totalFrames++;
            sleep(20); // 50Hz control loop
        }

        // Stop robot
        driveController.stopDrive();

        // Display success message
        if (isCentered) {
            displaySuccess();
        }

        // Wait before ending
        sleep(3000);
    }

    /**
     * Initialize hardware components
     */
    private void initializeHardware() {
        telemetry.addLine("=== INITIALIZING HARDWARE ===");
        telemetry.update();

        // Initialize drive system
        try {
            driveController = new SixWheelDriveController(this);
            telemetry.addLine("✓ Drive controller initialized");
        } catch (Exception e) {
            telemetry.addLine("ERROR initializing drive controller:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(5000);
            throw e;
        }

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (limelight != null) {
                limelight.pipelineSwitch(0); // AprilTag pipeline
                limelight.start();
                telemetry.addLine("✓ Limelight initialized");
            }
        } catch (Exception e) {
            limelight = null;
            telemetry.addLine("ERROR: Limelight not found!");
            telemetry.update();
            sleep(5000);
            throw e;
        }

        telemetry.addLine();
        telemetry.addLine("Hardware initialization complete!");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Update tracking data from Limelight
     */
    private void updateTracking() {
        if (limelight == null) {
            hasTarget = false;
            return;
        }

        try {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                hasTarget = false;
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        hasTarget = true;

                        // Get horizontal angle from Limelight
                        // tx > 0: target is to the right
                        // tx < 0: target is to the left
                        tx = fiducial.getTargetXDegrees();

                        break;
                    }
                }
            } else {
                hasTarget = false;
            }
        } catch (Exception e) {
            hasTarget = false;
            telemetry.addLine("Error reading Limelight: " + e.getMessage());
        }
    }

    /**
     * Perform rotation to center on AprilTag
     */
    private void performCentering() {
        double rotationPower = 0.0;

        // Check if we're outside the deadband
        if (Math.abs(tx) > CenteringParams.ROTATION_DEADBAND) {

            // Calculate proportional control
            // FIXED: Correct the rotation direction
            // When tx > 0 (target to right), we need to turn right (positive rotation)
            // When tx < 0 (target to left), we need to turn left (negative rotation)
            rotationPower = CenteringParams.KP_ROTATION * tx;

            // Add minimum power to overcome friction (maintaining sign)
            if (Math.abs(rotationPower) < CenteringParams.MIN_ROTATION_POWER) {
                rotationPower = Math.signum(rotationPower) * CenteringParams.MIN_ROTATION_POWER;
            }

            // Apply maximum speed limit
            rotationPower = Math.max(-CenteringParams.MAX_ROTATION_POWER,
                    Math.min(CenteringParams.MAX_ROTATION_POWER, rotationPower));
        }

        // Apply rotation using tank drive
        // For rotation in place:
        // Positive rotation (turn right): left forward, right backward
        // Negative rotation (turn left): left backward, right forward
        double leftPower = rotationPower;
        double rightPower = -rotationPower;

        driveController.tankDrive(leftPower, rightPower);
    }

    /**
     * Check if robot is centered and stable
     */
    private void checkCenteringStatus() {
        if (Math.abs(tx) <= CenteringParams.CENTERING_TOLERANCE) {
            // We're within tolerance
            if (centeredStartTime == 0) {
                // Just entered tolerance zone
                centeredStartTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - centeredStartTime > CenteringParams.STABLE_TIME_MS) {
                // We've been centered for long enough
                isCentered = true;
            }
        } else {
            // Outside tolerance - reset timer
            centeredStartTime = 0;
        }
    }

    /**
     * Display startup information
     */
    private void displayStartup() {
        telemetry.clear();
        telemetry.addLine("╔════════════════════════════════╗");
        telemetry.addLine("║   APRILTAG CENTERING AUTO      ║");
        telemetry.addLine("╚════════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("MISSION:");
        telemetry.addLine("  • Detect AprilTag ID " + TARGET_TAG_ID);
        telemetry.addLine("  • Rotate to center on tag");
        telemetry.addLine("  • No forward/backward movement");
        telemetry.addLine();
        telemetry.addLine("PARAMETERS:");
        telemetry.addData("  KP", "%.3f", CenteringParams.KP_ROTATION);
        telemetry.addData("  Tolerance", "%.1f°", CenteringParams.CENTERING_TOLERANCE);
        telemetry.addData("  Min Power", "%.2f", CenteringParams.MIN_ROTATION_POWER);
        telemetry.addData("  Max Power", "%.2f", CenteringParams.MAX_ROTATION_POWER);
        telemetry.addLine();
        telemetry.addLine(">>> Press START to begin <<<");
        telemetry.update();
    }

    /**
     * Display tracking information
     */
    private void displayTrackingInfo() {
        telemetry.clear();
        telemetry.addLine("═══ CENTERING ON TAG " + TARGET_TAG_ID + " ═══");
        telemetry.addLine();

        // Current angle
        telemetry.addData("TX (Angle)", "%.2f°", tx);

        // Visual indicator
        String indicator = getIndicator();
        telemetry.addLine("Direction: " + indicator);

        // Status
        if (Math.abs(tx) <= CenteringParams.CENTERING_TOLERANCE) {
            long timeInCenter = centeredStartTime > 0 ?
                    System.currentTimeMillis() - centeredStartTime : 0;
            telemetry.addLine();
            telemetry.addLine("✓ WITHIN TOLERANCE");
            telemetry.addData("Stable for", "%.1f sec", timeInCenter / 1000.0);
            telemetry.addData("Required", "%.1f sec", CenteringParams.STABLE_TIME_MS / 1000.0);
        } else {
            telemetry.addData("Error", "%.2f°", Math.abs(tx));
        }

        // Control info
        telemetry.addLine();
        telemetry.addLine("CONTROL:");
        telemetry.addData("  KP", "%.3f", CenteringParams.KP_ROTATION);
        telemetry.addData("  Tolerance", "%.1f°", CenteringParams.CENTERING_TOLERANCE);

        // Stats
        telemetry.addLine();
        telemetry.addData("Frames", "%d / %d", framesWithTarget, totalFrames);

        telemetry.update();
    }

    @NonNull
    private String getIndicator() {
        String indicator;
        if (tx < -5) {
            indicator = "<<<< LEFT";
        } else if (tx < -2) {
            indicator = "<< LEFT";
        } else if (tx < -CenteringParams.CENTERING_TOLERANCE) {
            indicator = "< LEFT";
        } else if (tx > 5) {
            indicator = "RIGHT >>>>";
        } else if (tx > 2) {
            indicator = "RIGHT >>";
        } else if (tx > CenteringParams.CENTERING_TOLERANCE) {
            indicator = "RIGHT >";
        } else {
            indicator = "[ CENTERED ]";
        }
        return indicator;
    }

    /**
     * Display no target message
     */
    private void displayNoTarget() {
        telemetry.clear();
        telemetry.addLine("═══ SEARCHING FOR TAG ═══");
        telemetry.addLine();
        telemetry.addLine("No AprilTag ID " + TARGET_TAG_ID + " detected");
        telemetry.addLine();
        telemetry.addLine("Please ensure:");
        telemetry.addLine("  • Tag is in camera view");
        telemetry.addLine("  • Adequate lighting");
        telemetry.addLine("  • Tag is not obstructed");
        telemetry.addLine();
        telemetry.addData("Frames checked", totalFrames);
        telemetry.update();
    }

    /**
     * Display success message
     */
    private void displaySuccess() {
        telemetry.clear();
        telemetry.addLine("╔════════════════════════════════╗");
        telemetry.addLine("║        ✓✓✓ SUCCESS ✓✓✓        ║");
        telemetry.addLine("╚════════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("Robot is centered on AprilTag " + TARGET_TAG_ID);
        telemetry.addData("Final angle", "%.2f°", tx);
        telemetry.addLine();
        telemetry.addData("Total frames", totalFrames);
        telemetry.addData("Frames with target", framesWithTarget);
        if (totalFrames > 0) {
            telemetry.addData("Detection rate", "%.1f%%",
                    100.0 * framesWithTarget / totalFrames);
        }
        telemetry.update();
    }

    /**
     * Send data to FTC Dashboard
     */
    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();

        // Tracking status
        packet.put("Has_Target", hasTarget);
        packet.put("TX_Angle", tx);
        packet.put("Is_Centered", isCentered);

        // Control parameters
        packet.put("KP_Rotation", CenteringParams.KP_ROTATION);
        packet.put("Tolerance", CenteringParams.CENTERING_TOLERANCE);
        packet.put("Min_Power", CenteringParams.MIN_ROTATION_POWER);
        packet.put("Max_Power", CenteringParams.MAX_ROTATION_POWER);

        // Odometry
        packet.put("Odo_X_mm", driveController.getX());
        packet.put("Odo_Y_mm", driveController.getY());
        packet.put("Odo_Heading_deg", driveController.getHeadingDegrees());

        // Stats
        packet.put("Frames_Total", totalFrames);
        packet.put("Frames_With_Target", framesWithTarget);

        dashboard.sendTelemetryPacket(packet);
    }
}