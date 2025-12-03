package org.firstinspires.ftc.teamcode.champion.controller;

import static java.lang.Thread.sleep;

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
 * Turret-based AprilTag alignment using the new TurretController
 * Identifies AprilTag and rotates turret towards it
 */
@Config
public class TurretAlignmentController {

    private final LinearOpMode opMode;
    private final TurretController turretController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    // === TUNABLE PARAMETERS ===

    @Config
    public static class AlignmentParams {
        public static double ANGLE_TOLERANCE_DEGREES = 2.0;  // Match TurretController tolerance
        public static long ALIGNMENT_TIMEOUT_MS = 3000;      // Match TurretController timeout
        public static int ALIGNED_FRAMES_REQUIRED = 5;
        public static double SETTLING_TIME_MS = 100;
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
    private double initialTurretPosition = 0;
    private double targetTurretPosition = 0;
    private boolean targetTurretPositionCalculated = false;

    // Alignment tracking
    private int consecutiveAlignedFrames = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();
    private double searchStartAngle = 0;

    // === INITIALIZATION ===

    public TurretAlignmentController(LinearOpMode opMode, TurretController turretController) throws Exception {
        this.opMode = opMode;
        this.turretController = turretController;
        this.dashboard = FtcDashboard.getInstance();

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.pipelineSwitch(0);
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
        targetTurretPositionCalculated = false;
        alignmentTimer.reset();

        // Update turret position
        turretController.update();
        initialTurretPosition = turretController.getCurrentAngle();

        opMode.telemetry.addLine("=== TURRET ALIGNMENT STARTED ===");
        opMode.telemetry.addData("Initial Turret", "%.2f°", initialTurretPosition);

        // Try to find target
        if (findTarget()) {
            // Calculate target turret position from initial tx
            targetTurretPosition = normalizeTurretAngle(initialTurretPosition + initialTx);
            targetTurretPositionCalculated = true;

            opMode.telemetry.addData("Initial TX", "%.2f°", initialTx);
            opMode.telemetry.addData("Target Turret", "%.2f°", targetTurretPosition);
            opMode.telemetry.update();

            // Use TurretController's built-in PID to move to target
            turretController.setTargetAngle(targetTurretPosition);
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
        } else {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
            searchStartAngle = initialTurretPosition;

            opMode.telemetry.addLine("Target not found - starting search");
            opMode.telemetry.update();
        }
    }

    /**
     * Stop alignment
     */
    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        turretController.stop();
    }

    /**
     * Main update method - call this in your main loop
     */
    public void update() {
        if (!isActive) return;

        turretController.update();

        switch (currentState) {
            case ALIGNING:
                updateAligning();
                break;
            case SEARCHING:
                updateSearching();
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
     * Update when aligning to target
     */
    private void updateAligning() {
        // Check if TurretController reached the target
        if (turretController.atTarget()) {
            consecutiveAlignedFrames++;

            if (consecutiveAlignedFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();

                opMode.telemetry.addLine("✓✓✓ ALIGNED! ✓✓✓");
                opMode.telemetry.update();
            }
        } else {
            consecutiveAlignedFrames = 0;
        }

        // Check for timeout
        if (alignmentTimer.milliseconds() > AlignmentParams.ALIGNMENT_TIMEOUT_MS) {
            opMode.telemetry.addLine("⚠️ Alignment timeout");
            opMode.telemetry.update();
            currentState = AlignmentState.TARGET_LOST;
            turretController.stop();
        }
    }

    /**
     * Update when searching for target
     */
    private void updateSearching() {
        if (!TargetSearch.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.stop();
            return;
        }

        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.stop();
            return;
        }

        // Try to find target during search
        if (findTarget()) {
            // Found target!
            turretController.update();
            initialTurretPosition = turretController.getCurrentAngle();

            targetTurretPosition = normalizeTurretAngle(initialTurretPosition + initialTx);
            targetTurretPositionCalculated = true;
            hasTarget = true;

            // Start alignment using TurretController
            turretController.setTargetAngle(targetTurretPosition);
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            return;
        }

        // Continue searching - slow rotation
        turretController.setPower(TargetSearch.SEARCH_SPEED);
    }

    /**
     * Maintain alignment (fine adjustments if needed)
     */
    public void maintainAlignment() {
        if (!targetTurretPositionCalculated) return;

        // TurretController's update() already maintains the target position
        // We just need to check if we've drifted too far
        double currentError = Math.abs(turretController.getAngleError());

        if (currentError > AlignmentParams.ANGLE_TOLERANCE_DEGREES * 2) {
            // Lost alignment, restart
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            turretController.setTargetAngle(targetTurretPosition);
        }
    }

    // === HELPER METHODS ===

    /**
     * Normalize angle to turret limits
     */
    private double normalizeTurretAngle(double angle) {
        // First normalize to -180 to 180
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;

        // Then clamp to turret limits
        if (TurretController.ENABLE_LIMITS) {
            angle = Math.max(TurretController.MIN_ANGLE,
                    Math.min(TurretController.MAX_ANGLE, angle));
        }

        return angle;
    }

    /**
     * Calculate the smallest angle difference between two angles (in degrees)
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
                    sleep(20);
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

        } catch (Exception ignored) {
        }

        hasTarget = false;
        return false;
    }

    // === TELEMETRY ===

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // State
        packet.put("State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("Target_Calculated", targetTurretPositionCalculated);

        // Angles
        double currentAngle = turretController.getCurrentAngle();
        packet.put("Initial_Turret", initialTurretPosition);
        packet.put("Current_Turret", currentAngle);
        packet.put("Target_Turret", targetTurretPosition);
        packet.put("Initial_TX", initialTx);

        if (targetTurretPositionCalculated) {
            double error = angleDifferenceDegrees(currentAngle, targetTurretPosition);
            packet.put("Angle_Error", error);
        }

        // Power
        packet.put("Turret_Power", turretController.getPower());

        // Timing
        packet.put("Aligned_Frames", consecutiveAlignedFrames);
        packet.put("Alignment_Time", alignmentTimer.seconds());
        if (totalAlignmentTime > 0) {
            packet.put("Total_Time", totalAlignmentTime);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayTelemetry() {
        opMode.telemetry.addLine("═══════════════════════");
        opMode.telemetry.addLine("  TURRET ALIGNMENT");
        opMode.telemetry.addLine("═══════════════════════");
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> INITIAL TX ANGLE <<<");
        opMode.telemetry.addData("TX", "%.2f°", initialTx);
        opMode.telemetry.addLine();

        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Current Angle", "%.2f°", turretController.getCurrentAngle());
        opMode.telemetry.addData("Target Angle", "%.2f°", targetTurretPosition);

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
        if (!targetTurretPositionCalculated) return 999;
        return Math.abs(angleDifferenceDegrees(turretController.getCurrentAngle(), targetTurretPosition));
    }

    public double getTotalAlignmentTime() {
        return totalAlignmentTime;
    }

    public void displayAlignmentWithInitialAngle() {
        opMode.telemetry.addLine("╔═══════════════════════╗");
        opMode.telemetry.addLine("║ TURRET ALIGNMENT      ║");
        opMode.telemetry.addLine("╚═══════════════════════╝");
        opMode.telemetry.addLine();

        opMode.telemetry.addLine(">>> INITIAL TX ANGLE <<<");
        opMode.telemetry.addData("TX", String.format(Locale.US, "%.2f°", initialTx));
        opMode.telemetry.addLine();

        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Current", String.format(Locale.US, "%.2f°", turretController.getCurrentAngle()));
        opMode.telemetry.addData("Target", String.format(Locale.US, "%.2f°", targetTurretPosition));

        opMode.telemetry.update();
    }
}