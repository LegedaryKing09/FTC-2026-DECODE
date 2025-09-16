package org.firstinspires.ftc.teamcode.champion.Auton;

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
 * Fixed Autonomous AprilTag Tracking
 * - NO SEARCHING - robot waits for tag to be visible
 * - Tracks tag when visible
 * - Maintains 48 inch distance
 * - Uses correct Limelight distance formula
 */
@Config
@Autonomous(name = "Track AprilTag (Fixed)", group = "Champion")
public class LimelightTankAutonomous extends LinearOpMode {

    // Hardware
    private SixWheelDriveController driveController;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    // Constants
    private static final int TARGET_TAG_ID = 2;        // Track tag 2
    private static final double TARGET_DISTANCE = 48.0; // 48 inches from tag
    private static final double TAG_HEIGHT = 29.5;      // AprilTag height from floor (inches)

    // Tunable parameters (adjust via FTC Dashboard)
    @Config
    public static class TrackingParams {
        // Camera mounting parameters - MEASURE THESE ON YOUR ROBOT!
        public static double CAMERA_HEIGHT = 14.8;      // Height from floor to camera lens (inches)
        public static double CAMERA_ANGLE = 7.9;      // Camera tilt angle (degrees from horizontal)

        // Control gains - Start conservative and increase as needed
        public static double KP_AIM = 0.02;             // Proportional gain for turning (positive!)
        public static double KP_DISTANCE = 0.015;       // Proportional gain for distance
        public static double MIN_AIM_COMMAND = 0.05;    // Minimum power to overcome friction

        // Tolerances
        public static double AIM_TOLERANCE = 2.0;       // Degrees - considered aligned
        public static double DISTANCE_TOLERANCE = 2.0;  // Inches - considered at distance

        // Speed limits
        public static double MAX_TURN_SPEED = 0.3;      // Maximum turning speed
        public static double MAX_DRIVE_SPEED = 0.35;    // Maximum forward/back speed

        // Behavior settings
        public static boolean ENABLE_SEARCH = false;    // Set to true to enable rotation search
        public static double SEARCH_SPEED = 0.3;       // Speed for rotation search (if enabled)
    }

    // Tracking data
    private double tx = 0;           // Horizontal angle to tag (degrees)
    private double ty = 0;           // Vertical angle to tag (degrees)
    private double calculatedDistance = 0;
    private double relativeX = 0;    // Lateral position from tag center (inches)
    private double relativeY = 0;    // Forward distance to tag (inches)
    private boolean hasTarget = false;
    private long trackingStartTime = 0;
    private int framesTracked = 0;
    private int framesWithoutTarget = 0;

    @Override
    public void runOpMode() {

        // Initialize hardware
        initializeHardware();

        // Display startup info
        displayStartup();

        waitForStart();

        if (isStopRequested()) return;

        trackingStartTime = System.currentTimeMillis();

        // Main tracking loop
        while (opModeIsActive()) {

            // Update tracking from Limelight
            updateTracking();

            if (hasTarget) {
                // We see the tag - track it!
                framesWithoutTarget = 0;
                performVisualServoing();
                displayTrackingInfo();
                framesTracked++;
            } else {
                // No tag visible
                framesWithoutTarget++;

                if (TrackingParams.ENABLE_SEARCH) {
                    // Only search if explicitly enabled
                    searchForTag();
                    displaySearching();
                } else {
                    // Default: Just stop and wait for tag to appear
                    driveController.stopDrive();
                    displayWaiting();
                }
            }

            // Update odometry
            driveController.updateOdometry();

            // Send data to dashboard
            sendDashboardData();

            sleep(20); // 50Hz control loop
        }

        // Stop robot
        driveController.stopDrive();

        // Display final statistics
        displayFinalStats();
    }

    /**
     * Initialize hardware components
     */
    private void initializeHardware() {
        telemetry.addLine("=== INITIALIZING HARDWARE ===");
        telemetry.update();

        // Initialize drive system
        try {
            driveController = new SixWheelDriveController();
            driveController.init(hardwareMap);
            telemetry.addLine("✓ Drive controller initialized");
        } catch (Exception e) {
            telemetry.addLine("ERROR initializing drive controller:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(10000);
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
            telemetry.addLine("WARNING: Limelight not found!");
        }

        telemetry.addLine();
        telemetry.addLine("Initialization complete!");
        telemetry.update();
        sleep(2000);
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

                        // Get angles from Limelight
                        tx = fiducial.getTargetXDegrees();
                        ty = fiducial.getTargetYDegrees();

                        // Calculate distance using CORRECT Limelight formula
                        calculatedDistance = calculateDistanceLimelight(ty);

                        // Calculate 3D position
                        calculate3DPosition();

                        break;
                    }
                }
            } else {
                hasTarget = false;
            }
        } catch (Exception e) {
            hasTarget = false;
        }
    }

    /**
     * Visual servoing control - FIXED version based on Limelight docs
     */
    private void performVisualServoing() {
        // Calculate errors
        double headingError = tx;  // Positive tx means target is to the right
        double distanceError = TARGET_DISTANCE - calculatedDistance; // Positive means too far

        // --- AIMING CONTROL ---
        double steeringAdjust = 0.0;

        if (Math.abs(headingError) > TrackingParams.AIM_TOLERANCE) {
            // Apply proportional control with minimum command
            steeringAdjust = TrackingParams.KP_AIM * headingError;

            // Add minimum command to overcome friction (following Limelight example)
            if (headingError > 0) {
                // Target is to the right, turn right
                steeringAdjust = Math.max(steeringAdjust, TrackingParams.MIN_AIM_COMMAND);
            } else {
                // Target is to the left, turn left
                steeringAdjust = Math.min(steeringAdjust, -TrackingParams.MIN_AIM_COMMAND);
            }
        }

        // --- DISTANCE CONTROL ---
        double distanceAdjust = 0.0;

        if (Math.abs(distanceError) > TrackingParams.DISTANCE_TOLERANCE) {
            // Positive error means we're too far, need to drive forward
            distanceAdjust = TrackingParams.KP_DISTANCE * distanceError;
        }

        // Apply speed limits
        steeringAdjust = Math.max(-TrackingParams.MAX_TURN_SPEED,
                Math.min(TrackingParams.MAX_TURN_SPEED, steeringAdjust));
        distanceAdjust = Math.max(-TrackingParams.MAX_DRIVE_SPEED,
                Math.min(TrackingParams.MAX_DRIVE_SPEED, distanceAdjust));

        // Apply to tank drive
        // For tank drive: left + steering turns right, right - steering turns right
        double leftPower = distanceAdjust - steeringAdjust;
        double rightPower = distanceAdjust + steeringAdjust;

        // Send to motors
        driveController.tankDrive(leftPower, rightPower);
    }

    /**
     * Search for tag by rotating (only if enabled)
     */
    private void searchForTag() {
        // Rotate slowly to search
        driveController.tankDrive(-TrackingParams.SEARCH_SPEED, TrackingParams.SEARCH_SPEED);
    }

    /**
     * Calculate distance using CORRECT Limelight formula
     * From Limelight docs: d = (h2-h1) / tan(a1+a2)
     * where:
     * - h1 = camera height
     * - h2 = target height
     * - a1 = camera mounting angle
     * - a2 = vertical angle to target (ty)
     */
    private double calculateDistanceLimelight(double ty) {
        // Convert angles to radians
        double angleToGoalDegrees = TrackingParams.CAMERA_ANGLE + ty;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        // Apply the Limelight distance formula
        double distance = (TAG_HEIGHT - TrackingParams.CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);

        // Return absolute value to handle any sign issues
        return Math.abs(distance);
    }

    /**
     * Calculate 3D position relative to tag
     */
    private void calculate3DPosition() {
        // Forward distance (Y) - this is our calculated distance
        relativeY = calculatedDistance;

        // Lateral offset (X) - how far left/right from tag center
        double txRadians = tx * (Math.PI / 180.0);
        relativeX = calculatedDistance * Math.tan(txRadians);
    }

    /**
     * Display waiting message when no tag is visible
     */
    private void displayWaiting() {
        telemetry.clear();
        telemetry.addLine("═══ WAITING FOR TAG ═══");
        telemetry.addLine();
        telemetry.addLine("Place AprilTag ID " + TARGET_TAG_ID + " in view");
        telemetry.addLine();
        telemetry.addLine("Robot will track when tag is visible");
        telemetry.addLine("Search mode: " + (TrackingParams.ENABLE_SEARCH ? "ENABLED" : "DISABLED"));
        telemetry.addLine();
        telemetry.addLine("Camera Settings:");
        telemetry.addData("  Height", "%.1f in", TrackingParams.CAMERA_HEIGHT);
        telemetry.addData("  Angle", "%.1f°", TrackingParams.CAMERA_ANGLE);
        telemetry.addLine();
        telemetry.addData("Frames without target", framesWithoutTarget);
        telemetry.update();
    }

    /**
     * Display searching message (only if search is enabled)
     */
    private void displaySearching() {
        telemetry.clear();
        telemetry.addLine("═══ SEARCHING FOR TAG " + TARGET_TAG_ID + " ═══");
        telemetry.addLine();
        telemetry.addLine("Rotating at speed: " + TrackingParams.SEARCH_SPEED);
        telemetry.addLine();
        telemetry.addData("Frames without target", framesWithoutTarget);
        telemetry.update();
    }

    /**
     * Display tracking information
     */
    private void displayTrackingInfo() {
        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║   TRACKING APRILTAG ID " + TARGET_TAG_ID + "     ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();

        // Current measurements
        telemetry.addLine("MEASUREMENTS:");
        telemetry.addData("  TX (bearing)", "%.2f°", tx);
        telemetry.addData("  TY (elevation)", "%.2f°", ty);
        telemetry.addData("  Distance", "%.1f inches", calculatedDistance);
        telemetry.addLine();

        // Target vs actual
        telemetry.addLine("TARGET STATUS:");
        telemetry.addData("  Distance", "%.1f / %.1f inches",
                calculatedDistance, TARGET_DISTANCE);

        // Errors
        double headingError = Math.abs(tx);
        double distanceError = Math.abs(calculatedDistance - TARGET_DISTANCE);

        if (headingError < TrackingParams.AIM_TOLERANCE) {
            telemetry.addLine("  ✓ AIMED");
        } else {
            telemetry.addData("  Aim Error", "%.1f°", headingError);
        }

        if (distanceError < TrackingParams.DISTANCE_TOLERANCE) {
            telemetry.addLine("  ✓ AT DISTANCE");
        } else {
            telemetry.addData("  Distance Error", "%.1f inches", distanceError);
        }

        // Perfect alignment indicator
        if (headingError < TrackingParams.AIM_TOLERANCE &&
                distanceError < TrackingParams.DISTANCE_TOLERANCE) {
            telemetry.addLine();
            telemetry.addLine("════════════════════════");
            telemetry.addLine("  ✓✓✓ ON TARGET ✓✓✓");
            telemetry.addLine("════════════════════════");
        }

        // Control parameters
        telemetry.addLine();
        telemetry.addLine("CONTROL:");
        telemetry.addData("  KP Aim", "%.3f", TrackingParams.KP_AIM);
        telemetry.addData("  KP Distance", "%.3f", TrackingParams.KP_DISTANCE);

        // Stats
        double runtime = (System.currentTimeMillis() - trackingStartTime) / 1000.0;
        telemetry.addLine();
        telemetry.addData("Runtime", "%.1f sec", runtime);
        telemetry.addData("Frames Tracked", framesTracked);

        telemetry.update();
    }

    /**
     * Display startup information
     */
    private void displayStartup() {
        telemetry.clear();
        telemetry.addLine("╔════════════════════════════════╗");
        telemetry.addLine("║  APRILTAG TRACKER (FIXED)     ║");
        telemetry.addLine("╚════════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("BEHAVIOR:");
        if (TrackingParams.ENABLE_SEARCH) {
            telemetry.addLine("  • Will rotate to search for tag");
        } else {
            telemetry.addLine("  • Will wait for tag to be visible");
            telemetry.addLine("  • No automatic searching");
        }
        telemetry.addLine();
        telemetry.addLine("TARGET:");
        telemetry.addData("  Tag ID", TARGET_TAG_ID);
        telemetry.addData("  Distance", "%.0f inches", TARGET_DISTANCE);
        telemetry.addLine();
        telemetry.addLine("CAMERA CALIBRATION:");
        telemetry.addData("  Height", "%.1f inches", TrackingParams.CAMERA_HEIGHT);
        telemetry.addData("  Angle", "%.1f degrees", TrackingParams.CAMERA_ANGLE);
        telemetry.addLine();
        telemetry.addLine(">>> Press START to begin <<<");
        telemetry.update();
    }

    /**
     * Display final statistics
     */
    private void displayFinalStats() {
        double runtime = (System.currentTimeMillis() - trackingStartTime) / 1000.0;

        telemetry.clear();
        telemetry.addLine("═══ SESSION COMPLETE ═══");
        telemetry.addLine();
        telemetry.addData("Total Runtime", "%.1f seconds", runtime);
        telemetry.addData("Frames Tracked", framesTracked);
        if (framesTracked > 0) {
            telemetry.addData("Tracking Rate", "%.1f fps", framesTracked / runtime);
            telemetry.addLine();
            telemetry.addLine("Final Position:");
            telemetry.addData("  Distance", "%.1f inches", calculatedDistance);
            telemetry.addData("  Bearing", "%.1f degrees", tx);
        } else {
            telemetry.addLine();
            telemetry.addLine("No tracking data collected");
        }
        telemetry.update();

        sleep(5000);
    }

    /**
     * Send data to FTC Dashboard
     */
    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();

        // Tracking status
        packet.put("Has_Target", hasTarget);
        packet.put("TX_Angle", tx);
        packet.put("TY_Angle", ty);
        packet.put("Distance", calculatedDistance);

        // Errors
        packet.put("Distance_Error", calculatedDistance - TARGET_DISTANCE);
        packet.put("Heading_Error", tx);

        // Control params
        packet.put("KP_Aim", TrackingParams.KP_AIM);
        packet.put("KP_Distance", TrackingParams.KP_DISTANCE);

        // Odometry
        packet.put("Odo_X_mm", driveController.getX());
        packet.put("Odo_Y_mm", driveController.getY());
        packet.put("Odo_Heading_deg", driveController.getHeadingDegrees());

        dashboard.sendTelemetryPacket(packet);
    }
}