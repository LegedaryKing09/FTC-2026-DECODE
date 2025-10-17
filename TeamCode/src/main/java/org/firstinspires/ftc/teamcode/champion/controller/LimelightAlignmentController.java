package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import java.util.List;

/**
 * Odometry-based AprilTag alignment
 * Reads initial tx from AprilTag, calculates target heading using odometry, then turns to it
 */
@Config
public class LimelightAlignmentController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    // === TUNABLE PARAMETERS ===

    @Config
    public static class AlignmentParams {
        public static double HEADING_TOLERANCE_DEGREES = 1.0;  // Increased from 0.5
        public static double TURN_SPEED = 0.3;  // Reduced from 0.4
        public static double MIN_TURN_SPEED = 0.1;  // Reduced from 0.15
        public static double SLOW_DOWN_ANGLE = 20.0;  // Increased from 15.0
        public static long ALIGNMENT_TIMEOUT_MS = 5000;
        public static int ALIGNED_FRAMES_REQUIRED = 5;  // Reduced from 10
    }

    @Config
    public static class VelocityTuning {
        public static boolean USE_VELOCITY_MODE = true;  // Toggle between velocity and power
        public static double ANGULAR_VELOCITY_MAX = 80.0;  // degrees/sec (reduced from 120)
        public static double ANGULAR_VELOCITY_MIN = 15.0;   // degrees/sec
    }

    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_SPEED = 0.3;
        public static long MAX_SEARCH_TIME_MS = 5000;
    }

    // === STATE VARIABLES ===

    public enum AlignmentState {
        ALIGNING, ALIGNED, TARGET_LOST, SEARCHING, STOPPED
    }

    private AlignmentState currentState = AlignmentState.STOPPED;
    private boolean isActive = false;

    // Target tracking
    private int targetTagId = 20;
    private boolean hasTarget = false;
    private double initialTx = 0;
    private double initialHeadingDegrees = 0;
    private double targetHeadingDegrees = 0;
    private boolean targetHeadingCalculated = false;

    // Alignment tracking
    private int consecutiveAlignedFrames = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();

    // === INITIALIZATION ===

    public LimelightAlignmentController(LinearOpMode opMode, SixWheelDriveController driveController) throws Exception {
        this.opMode = opMode;
        this.driveController = driveController;

        this.dashboard = FtcDashboard.getInstance();

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.pipelineSwitch(1);
            this.limelight.start();
        } catch (Exception e) {
            throw new Exception("Failed to initialize Limelight: " + e.getMessage());
        }
    }

    // === PUBLIC API ===

    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    public void startAlignment() {
        isActive = true;
        targetHeadingCalculated = false;
        alignmentTimer.reset();

        // Update odometry and get initial heading
        driveController.updateOdometry();
        initialHeadingDegrees = driveController.getHeadingDegrees();

        // Try to find target
        if (findTarget()) {
            // Calculate target heading from initial tx
            // If TX is positive (target LEFT), turn CCW (positive) to face it
            // If TX is negative (target RIGHT), turn CW (negative) to face it
            targetHeadingDegrees = normalizeAngle(initialHeadingDegrees + initialTx);
            targetHeadingCalculated = true;

            opMode.telemetry.addLine("=== ALIGNMENT STARTED ===");
            opMode.telemetry.addData("Initial Heading", "%.2f°", initialHeadingDegrees);
            opMode.telemetry.addData("AprilTag TX", "%.2f°", initialTx);
            opMode.telemetry.addData("TX Meaning", initialTx > 0 ? "Target LEFT" : "Target RIGHT");
            opMode.telemetry.addData("Target Heading", "%.2f°", targetHeadingDegrees);
            opMode.telemetry.update();

            // Execute the turn directly - BLOCKING METHOD
            try {
                turnToHeading(targetHeadingDegrees);
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
        } else {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();

            opMode.telemetry.addLine("Target not found - starting search");
            opMode.telemetry.update();
        }

        consecutiveAlignedFrames = 0;
    }

    /**
     * Turn to a target heading using odometry feedback
     * This is a BLOCKING method that returns when aligned or timeout
     */
    private void turnToHeading(double targetHeadingDegrees) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        int stableFrames = 0;

        while (opMode.opModeIsActive()) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > AlignmentParams.ALIGNMENT_TIMEOUT_MS) {
                opMode.telemetry.addLine("⚠️ Alignment timeout");
                opMode.telemetry.update();
                break;
            }

            // Update odometry
            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();

            // Calculate error
            double headingError = angleDifferenceDegrees(currentHeading, targetHeadingDegrees);
            double absError = Math.abs(headingError);

            // Display current status
            opMode.telemetry.addLine("=== TURNING TO TARGET ===");
            opMode.telemetry.addData("Current", "%.2f°", currentHeading);
            opMode.telemetry.addData("Target", "%.2f°", targetHeadingDegrees);
            opMode.telemetry.addData("Error", "%.2f°", headingError);
            opMode.telemetry.addData("Abs Error", "%.2f°", absError);
            opMode.telemetry.addData("Tolerance", "%.2f°", AlignmentParams.HEADING_TOLERANCE_DEGREES);
            opMode.telemetry.addData("Stable Frames", "%d/%d", stableFrames, AlignmentParams.ALIGNED_FRAMES_REQUIRED);

            // Check if aligned
            if (absError <= AlignmentParams.HEADING_TOLERANCE_DEGREES) {
                stableFrames++;
                opMode.telemetry.addLine("✓ Within tolerance - holding...");

                if (stableFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                    opMode.telemetry.addLine("✓✓✓ ALIGNED! ✓✓✓");
                    opMode.telemetry.update();
                    driveController.stopDrive();
                    Thread.sleep(100);
                    break;
                }

                // Stop while counting stable frames
                driveController.stopDrive();
            } else {
                // Reset stable counter if we drift
                stableFrames = 0;

                // Calculate turn speed based on error
                double turnSpeed;
                if (VelocityTuning.USE_VELOCITY_MODE) {
                    // Use velocity control
                    double angularVelocity;
                    if (absError > AlignmentParams.SLOW_DOWN_ANGLE) {
                        angularVelocity = VelocityTuning.ANGULAR_VELOCITY_MAX;
                    } else {
                        // Proportional scaling in slow zone
                        double scale = absError / AlignmentParams.SLOW_DOWN_ANGLE;
                        angularVelocity = VelocityTuning.ANGULAR_VELOCITY_MIN +
                                (VelocityTuning.ANGULAR_VELOCITY_MAX - VelocityTuning.ANGULAR_VELOCITY_MIN) * scale;
                    }
                    angularVelocity = Math.copySign(angularVelocity, headingError);

                    // FIXED: Don't negate! Positive error = CCW = positive velocity
                    driveController.setAngularVelocity(angularVelocity);

                    opMode.telemetry.addData("Angular Vel", "%.1f °/s", angularVelocity);
                } else {
                    // Use power control
                    if (absError > AlignmentParams.SLOW_DOWN_ANGLE) {
                        turnSpeed = AlignmentParams.TURN_SPEED;
                    } else {
                        // Proportional scaling
                        double scale = absError / AlignmentParams.SLOW_DOWN_ANGLE;
                        turnSpeed = AlignmentParams.MIN_TURN_SPEED +
                                (AlignmentParams.TURN_SPEED - AlignmentParams.MIN_TURN_SPEED) * scale;
                    }
                    turnSpeed = Math.copySign(turnSpeed, headingError);
                    driveController.tankDrive(-turnSpeed, turnSpeed);

                    opMode.telemetry.addData("Turn Power", "%.3f", turnSpeed);
                }
            }

            opMode.telemetry.addData("Left Vel", "%.0f", driveController.getLeftVelocity());
            opMode.telemetry.addData("Right Vel", "%.0f", driveController.getRightVelocity());
            opMode.telemetry.update();

            // Small delay for loop timing
            Thread.sleep(20);
        }

        // Ensure stopped
        driveController.stopDrive();
    }

    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        driveController.stopDrive();
    }

    public void align(int targetTagId) {
        if (!isActive) return;

        this.targetTagId = targetTagId;

        switch (currentState) {
            case SEARCHING:
                executeSearch();
                break;
            case ALIGNING:
                // Alignment is handled in startAlignment() now
                break;
            case ALIGNED:
                maintainAlignment();
                break;
            case TARGET_LOST:
            case STOPPED:
                break;
        }

        sendDashboardData();
    }

    /**
     * Normalize angle to [-180, 180] range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // === CORE ALIGNMENT LOGIC ===

    private void executeSearch() {
        if (!TargetSearch.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            driveController.stopDrive();
            return;
        }

        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            driveController.stopDrive();
            return;
        }

        // Try to find target during search
        if (findTarget()) {
            // Found target! Calculate heading and start alignment
            driveController.updateOdometry();
            initialHeadingDegrees = driveController.getHeadingDegrees();
            // If TX is positive (target LEFT), turn CCW (positive)
            targetHeadingDegrees = normalizeAngle(initialHeadingDegrees + initialTx);
            targetHeadingCalculated = true;

            try {
                turnToHeading(targetHeadingDegrees);
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
            return;
        }

        // Continue searching - slow rotation
        driveController.tankDrive(-TargetSearch.SEARCH_SPEED, TargetSearch.SEARCH_SPEED);
    }

    private void maintainAlignment() {
        // Update odometry to check if we've drifted
        driveController.updateOdometry();
        double currentHeadingDegrees = driveController.getHeadingDegrees();
        double headingError = angleDifferenceDegrees(currentHeadingDegrees, targetHeadingDegrees);
        double absError = Math.abs(headingError);

        // If error grows beyond dead zone, realign
        if (absError > AlignmentParams.HEADING_TOLERANCE_DEGREES * 3) {
            try {
                turnToHeading(targetHeadingDegrees);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return;
        }

        // Stay stopped
        driveController.stopDrive();
    }

    // === HELPER METHODS ===

    /**
     * Find the target AprilTag and read its TX value
     */
    private boolean findTarget() {
        if (limelight == null) {
            return false;
        }

        try {
            // Take multiple readings for better accuracy
            double sumTx = 0;
            int validReadings = 0;

            for (int i = 0; i < 5; i++) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == targetTagId) {
                            sumTx += fiducial.getTargetXDegrees();
                            validReadings++;
                            hasTarget = true;
                            break;
                        }
                    }
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return false;
                }
            }

            if (validReadings > 0) {
                initialTx = sumTx / validReadings;
                initialTx = -initialTx;  // Invert TX sign
                hasTarget = true;
                return true;
            }

        } catch (Exception e) {
            hasTarget = false;
        }

        hasTarget = false;
        return false;
    }

    /**
     * Calculate the smallest angle difference between two angles (in degrees)
     * Returns positive if target is CCW from current, negative if CW
     */
    private double angleDifferenceDegrees(double currentDegrees, double targetDegrees) {
        double diff = targetDegrees - currentDegrees;

        // Normalize to [-180, 180]
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;

        return diff;
    }

    // === TELEMETRY ===

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // State
        packet.put("State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("Target_Calculated", targetHeadingCalculated);

        // Headings
        driveController.updateOdometry();
        double currentHeading = driveController.getHeadingDegrees();
        packet.put("Initial_Heading", initialHeadingDegrees);
        packet.put("Current_Heading", currentHeading);
        packet.put("Target_Heading", targetHeadingDegrees);
        packet.put("Initial_TX", initialTx);

        if (targetHeadingCalculated) {
            double error = angleDifferenceDegrees(currentHeading, targetHeadingDegrees);
            packet.put("Heading_Error", error);
        }

        // Velocities
        packet.put("Left_Wheel_Vel", driveController.getLeftVelocity());
        packet.put("Right_Wheel_Vel", driveController.getRightVelocity());

        // Timing
        packet.put("Aligned_Frames", consecutiveAlignedFrames);
        packet.put("Alignment_Time", alignmentTimer.seconds());
        if (totalAlignmentTime > 0) {
            packet.put("Total_Time", totalAlignmentTime);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayAlignmentWithInitialAngle() {
        opMode.telemetry.addLine("╔═══════════════════════════╗");
        opMode.telemetry.addLine("║  ALIGNMENT STATUS         ║");
        opMode.telemetry.addLine("╚═══════════════════════════╝");

        opMode.telemetry.addLine();
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target Found", hasTarget ? "✓ YES" : "✗ NO");
        opMode.telemetry.addData("Mode", VelocityTuning.USE_VELOCITY_MODE ? "VELOCITY" : "POWER");

        if (targetHeadingCalculated) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine("┌─── HEADING DATA ───┐");
            opMode.telemetry.addData("│ Initial Heading", String.format(Locale.US, "%.2f°", initialHeadingDegrees));
            opMode.telemetry.addData("│ AprilTag TX", String.format(Locale.US, "%.2f°", initialTx));
            opMode.telemetry.addData("│ Target Heading", String.format(Locale.US, "%.2f°", targetHeadingDegrees));
            opMode.telemetry.addLine("└───────────────────┘");

            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();
            double error = angleDifferenceDegrees(currentHeading, targetHeadingDegrees);

            opMode.telemetry.addLine();
            opMode.telemetry.addLine("─── CURRENT STATUS ───");
            opMode.telemetry.addData("Current Heading", String.format(Locale.US, "%.2f°", currentHeading));
            opMode.telemetry.addData("Heading Error", String.format(Locale.US, "%.2f°", error));
            opMode.telemetry.addData("Abs Error", String.format(Locale.US, "%.2f°", Math.abs(error)));
            opMode.telemetry.addData("Aligned Frames", String.format(Locale.US, "%d/%d",
                    consecutiveAlignedFrames, AlignmentParams.ALIGNED_FRAMES_REQUIRED));

            // Debug: Show if error is within tolerance
            boolean withinTolerance = Math.abs(error) <= AlignmentParams.HEADING_TOLERANCE_DEGREES;
            opMode.telemetry.addData("Within Tolerance?", withinTolerance ? "YES ✓" : "NO");

            double progress = 0;
            if (Math.abs(initialTx) > 0.1) {
                progress = Math.max(0, Math.min(100, (1 - Math.abs(error) / Math.abs(initialTx)) * 100));
            }
            opMode.telemetry.addData("Progress", String.format(Locale.US, "%.0f%%", progress));
        }

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("─── WHEEL VELOCITIES ───");
        opMode.telemetry.addData("Left", String.format(Locale.US, "%.0f ticks/s", driveController.getLeftVelocity()));
        opMode.telemetry.addData("Right", String.format(Locale.US, "%.0f ticks/s", driveController.getRightVelocity()));

        if (currentState == AlignmentState.ALIGNED && totalAlignmentTime > 0) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine("╔═══════════════════════════╗");
            opMode.telemetry.addData("║ ✓ ALIGNED", String.format(Locale.US, "%.2fs", totalAlignmentTime));
            if (targetHeadingCalculated) {
                opMode.telemetry.addData("║ From", String.format(Locale.US, "%.2f° → %.2f°",
                        initialHeadingDegrees, targetHeadingDegrees));
            }
            opMode.telemetry.addLine("╚═══════════════════════════╝");
        }

        opMode.telemetry.update();
    }

    public void displayTelemetry() {
        opMode.telemetry.addLine("═══ ALIGNMENT STATUS ═══");
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target Found", hasTarget ? "YES" : "NO");
        opMode.telemetry.addData("Mode", VelocityTuning.USE_VELOCITY_MODE ? "VELOCITY" : "POWER");

        if (targetHeadingCalculated) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine("═══ HEADINGS ═══");
            opMode.telemetry.addData("Initial", "%.2f°", initialHeadingDegrees);
            opMode.telemetry.addData("Target", "%.2f°", targetHeadingDegrees);

            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();
            double error = angleDifferenceDegrees(currentHeading, targetHeadingDegrees);

            opMode.telemetry.addData("Current", "%.2f°", currentHeading);
            opMode.telemetry.addData("Error", "%.2f°", error);
        }

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("═══ VELOCITIES ═══");
        opMode.telemetry.addData("Left Wheel", "%.0f ticks/s", driveController.getLeftVelocity());
        opMode.telemetry.addData("Right Wheel", "%.0f ticks/s", driveController.getRightVelocity());

        if (currentState == AlignmentState.ALIGNED && totalAlignmentTime > 0) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("✓ Aligned in", "%.2fs", totalAlignmentTime);
        }

        opMode.telemetry.update();
    }

    // === GETTERS ===

    public boolean isAligned() {
        return currentState == AlignmentState.ALIGNED;
    }

    public boolean isSearching() {
        return currentState == AlignmentState.SEARCHING;
    }

    public AlignmentState getState() {
        return currentState;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTargetError() {
        if (!targetHeadingCalculated) return 999;
        driveController.updateOdometry();
        return Math.abs(angleDifferenceDegrees(driveController.getHeadingDegrees(), targetHeadingDegrees));
    }

    public double getTotalAlignmentTime() {
        return totalAlignmentTime;
    }

    public String getCurrentZone() {
        return "Odometry-Based";
    }
}