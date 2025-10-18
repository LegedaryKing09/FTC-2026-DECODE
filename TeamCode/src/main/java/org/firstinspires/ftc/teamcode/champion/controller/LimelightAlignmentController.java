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
 * Odometry-based AprilTag alignment with PID control
 * Uses PID to calculate angular velocity based on heading error
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
        public static double HEADING_TOLERANCE_DEGREES = 1.5;
        public static long ALIGNMENT_TIMEOUT_MS = 5000;
        public static int ALIGNED_FRAMES_REQUIRED = 5;
        public static double SETTLING_TIME_MS = 100; // Time to wait after stopping
    }

    @Config
    public static class PIDParams {
        // PID Gains - tune these for your robot
        public static double KP = 5;      // Proportional gain
        public static double KI = 0;      // Integral gain
        public static double KD = 0;      // Derivative gain

        // Output limits (degrees/sec)
        public static double MAX_ANGULAR_VELOCITY = 90.0;
        public static double MIN_ANGULAR_VELOCITY = 10.0;

        // Anti-windup
        public static double INTEGRAL_MAX = 30.0;  // Max integral accumulation

        // Deadband
        public static double DEADBAND_DEGREES = 0.5;  // Don't move if error is this small
    }

    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_SPEED = 0.4;
        public static long MAX_SEARCH_TIME_MS = 5000;
    }

    // === PID CONTROLLER CLASS ===

    private class PIDController {
        private double setpoint = 0;
        private double lastError = 0;
        private double integral = 0;
        private ElapsedTime timer = new ElapsedTime();
        private boolean firstUpdate = true;

        public void reset() {
            lastError = 0;
            integral = 0;
            firstUpdate = true;
            timer.reset();
        }

        public void setTarget(double targetHeading) {
            this.setpoint = targetHeading;
            reset();
        }

        public double calculate(double currentHeading) {
            // Calculate error (shortest path)
            double error = angleDifferenceDegrees(currentHeading, setpoint);

            // Apply deadband
            if (Math.abs(error) < PIDParams.DEADBAND_DEGREES) {
                return 0;
            }

            // Get time delta
            double dt = firstUpdate ? 0.02 : timer.seconds();
            timer.reset();

            if (firstUpdate) {
                firstUpdate = false;
                lastError = error;
            }

            // P term
            double pTerm = PIDParams.KP * error;

            // I term with anti-windup
            integral += error * dt;
            integral = Math.max(-PIDParams.INTEGRAL_MAX,
                    Math.min(PIDParams.INTEGRAL_MAX, integral));
            double iTerm = PIDParams.KI * integral;

            // D term
            double derivative = dt > 0 ? (error - lastError) / dt : 0;
            double dTerm = PIDParams.KD * derivative;

            // Calculate total output
            double output = pTerm + iTerm + dTerm;

            // Apply output limits
            output = Math.max(-PIDParams.MAX_ANGULAR_VELOCITY,
                    Math.min(PIDParams.MAX_ANGULAR_VELOCITY, output));

            // Apply minimum speed if we're still moving
            if (Math.abs(output) > 0 && Math.abs(output) < PIDParams.MIN_ANGULAR_VELOCITY) {
                output = Math.copySign(PIDParams.MIN_ANGULAR_VELOCITY, output);
            }

            // Store for next iteration
            lastError = error;

            // Debug telemetry
            opMode.telemetry.addLine("--- PID Debug ---");
            opMode.telemetry.addData("Error", "%.2f°", error);
            opMode.telemetry.addData("P", "%.2f", pTerm);
            opMode.telemetry.addData("I", "%.2f", iTerm);
            opMode.telemetry.addData("D", "%.2f", dTerm);
            opMode.telemetry.addData("Output", "%.2f°/s", output);

            return output;
        }

        public double getError(double currentHeading) {
            return angleDifferenceDegrees(currentHeading, setpoint);
        }

        public boolean isAtSetpoint(double currentHeading) {
            return Math.abs(getError(currentHeading)) <= AlignmentParams.HEADING_TOLERANCE_DEGREES;
        }
    }

    // === STATE VARIABLES ===

    public enum AlignmentState {
        ALIGNING, ALIGNED, TARGET_LOST, SEARCHING, STOPPED
    }

    private AlignmentState currentState = AlignmentState.STOPPED;
    private boolean isActive = false;

    // PID controller instance
    private final PIDController pidController = new PIDController();

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

    public LimelightAlignmentController(LinearOpMode opMode) throws Exception {
        this.opMode = opMode;

        try {
            this.driveController = new SixWheelDriveController(opMode);
        } catch (Exception e) {
            throw new Exception("Failed to initialize drive controller: " + e.getMessage());
        }

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
            targetHeadingDegrees = normalizeAngle(initialHeadingDegrees + initialTx);
            targetHeadingCalculated = true;

            // Initialize PID controller with target
            pidController.setTarget(targetHeadingDegrees);

            opMode.telemetry.addLine("=== ALIGNMENT STARTED ===");
            opMode.telemetry.addData("Initial Heading", "%.2f°", initialHeadingDegrees);
            opMode.telemetry.addData("AprilTag TX", "%.2f°", initialTx);
            opMode.telemetry.addData("TX Meaning", initialTx > 0 ? "Target LEFT" : "Target RIGHT");
            opMode.telemetry.addData("Target Heading", "%.2f°", targetHeadingDegrees);
            opMode.telemetry.update();

            // Execute PID-controlled turn
            try {
                turnToHeadingPID(targetHeadingDegrees);
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
     * Turn to target heading using PID control
     * This is a BLOCKING method that returns when aligned or timeout
     */
    private void turnToHeadingPID(double targetHeadingDegrees) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        int stableFrames = 0;

        // Reset PID controller
        pidController.setTarget(targetHeadingDegrees);

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

            // Calculate PID output (angular velocity in degrees/sec)
            double angularVelocity = pidController.calculate(currentHeading);

            // Get current error for display
            double headingError = pidController.getError(currentHeading);
            double absError = Math.abs(headingError);

            // Display status
            opMode.telemetry.addLine("=== PID ALIGNMENT ===");
            opMode.telemetry.addData("Current", "%.2f°", currentHeading);
            opMode.telemetry.addData("Target", "%.2f°", targetHeadingDegrees);
            opMode.telemetry.addData("Error", "%.2f°", headingError);
            opMode.telemetry.addData("Tolerance", "%.2f°", AlignmentParams.HEADING_TOLERANCE_DEGREES);
            opMode.telemetry.addData("Angular Vel Cmd", "%.1f°/s", angularVelocity);
            opMode.telemetry.addData("Stable Frames", "%d/%d", stableFrames, AlignmentParams.ALIGNED_FRAMES_REQUIRED);

            // Check if aligned
            if (pidController.isAtSetpoint(currentHeading)) {
                stableFrames++;
                opMode.telemetry.addLine("✓ Within tolerance - holding...");

                if (stableFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                    opMode.telemetry.addLine("✓✓✓ ALIGNED! ✓✓✓");
                    opMode.telemetry.update();
                    driveController.stopDrive();
                    Thread.sleep((long)AlignmentParams.SETTLING_TIME_MS);
                    break;
                }

                // Stop while counting stable frames
                driveController.stopDrive();
            } else {
                // Reset stable counter if we drift
                stableFrames = 0;

                // Apply the PID-calculated angular velocity
                driveController.setAngularVelocity(angularVelocity);
            }

            // Display actual wheel velocities
            opMode.telemetry.addData("Left Vel", "%.0f ticks/s", driveController.getLeftVelocity());
            opMode.telemetry.addData("Right Vel", "%.0f ticks/s", driveController.getRightVelocity());
            opMode.telemetry.update();

            // Loop timing (50Hz)
            Thread.sleep(20);
        }

        // Ensure stopped
        driveController.stopDrive();
    }

    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;
        pidController.reset();

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        driveController.stopDrive();
    }

    public void maintainAlignment() {
        if (!targetHeadingCalculated) return;

        // Continuously apply PID to maintain heading
        driveController.updateOdometry();
        double currentHeading = driveController.getHeadingDegrees();

        // Use PID to calculate correction
        double angularVelocity = pidController.calculate(currentHeading);

        // Apply correction if needed
        if (Math.abs(angularVelocity) > PIDParams.DEADBAND_DEGREES) {
            driveController.setAngularVelocity(angularVelocity);
        } else {
            driveController.stopDrive();
        }
    }

    // === HELPER METHODS ===

    /**
     * Normalize angle to [-180, 180] range
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
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
            targetHeadingDegrees = normalizeAngle(initialHeadingDegrees + initialTx);
            targetHeadingCalculated = true;
            hasTarget = true;

            try {
                turnToHeadingPID(targetHeadingDegrees);
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

    public void align(int targetTagId) {
        if (!isActive) return;

        this.targetTagId = targetTagId;

        switch (currentState) {
            case SEARCHING:
                executeSearch();
                break;
            case ALIGNED:
                maintainAlignment();
                break;
            case ALIGNING:
            case TARGET_LOST:
            case STOPPED:
                break;
        }

        sendDashboardData();
    }

    // === TELEMETRY ===

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // State
        packet.put("State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("Target_Calculated", targetHeadingCalculated);

        // PID parameters
        packet.put("PID_KP", PIDParams.KP);
        packet.put("PID_KI", PIDParams.KI);
        packet.put("PID_KD", PIDParams.KD);

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

    public void displayTelemetry() {
        opMode.telemetry.addLine("═══ PID ALIGNMENT STATUS ═══");
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target Found", hasTarget ? "YES" : "NO");

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("═══ PID GAINS ═══");
        opMode.telemetry.addData("KP", "%.2f", PIDParams.KP);
        opMode.telemetry.addData("KI", "%.2f", PIDParams.KI);
        opMode.telemetry.addData("KD", "%.2f", PIDParams.KD);

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
            opMode.telemetry.addData("Within Tolerance?",
                    Math.abs(error) <= AlignmentParams.HEADING_TOLERANCE_DEGREES ? "YES ✓" : "NO");
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

    public void displayAlignmentWithInitialAngle() {
        opMode.telemetry.addLine("╔═══════════════════════════╗");
        opMode.telemetry.addLine("║  ALIGNMENT STATUS         ║");
        opMode.telemetry.addLine("╚═══════════════════════════╝");

        opMode.telemetry.addLine();
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target Found", hasTarget ? "✓ YES" : "✗ NO");
        opMode.telemetry.addData("PID Mode", "ACTIVE");

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
        opMode.telemetry.addLine("─── PID STATUS ───");
        opMode.telemetry.addData("KP", "%.2f", PIDParams.KP);
        opMode.telemetry.addData("KI", "%.2f", PIDParams.KI);
        opMode.telemetry.addData("KD", "%.2f", PIDParams.KD);

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
}