package org.firstinspires.ftc.teamcode.champion.controller;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

/**
 * Simplified Turret-based AprilTag alignment
 * Turns at fixed power until TX is within tolerance
 */
@Config
public class TurretAlignmentController {

    private final LinearOpMode opMode;
    private final TurretController turretController;
    private final Limelight3A limelight;

    // === TUNABLE PARAMETERS ===

    @Config
    public static class AlignmentParams {
        public static double TX_TOLERANCE_DEGREES = 1.0;
        public static long ALIGNMENT_TIMEOUT_MS = 3000;
        public static int ALIGNED_FRAMES_REQUIRED = 5;
        public static double SETTLING_TIME_MS = 100;
        public static double TURN_POWER = 1.0;
    }

    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_POWER = 0.3;
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
    private double currentTx = 0;

    // Alignment tracking
    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();

    // === INITIALIZATION ===

    public TurretAlignmentController(LinearOpMode opMode, TurretController turretController) throws Exception {
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

    // === PUBLIC API ===

    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    public void startAlignment() {
        isActive = true;
        alignmentTimer.reset();

        // Try to find target
        if (findTarget()) {
            opMode.telemetry.addLine("=== TURRET ALIGNMENT STARTED ===");
            opMode.telemetry.addData("Initial TX", "%.2f°", currentTx);
            opMode.telemetry.update();

            // Execute alignment
            try {
                turnTurretToAlignWithTarget();
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
    }

    /**
     * Turn turret until TX is within tolerance
     * This is a BLOCKING method that returns when aligned or timeout
     * @noinspection BusyWait
     */
    private void turnTurretToAlignWithTarget() throws InterruptedException {
        long startTime = System.currentTimeMillis();
        int stableFrames = 0;

        while (opMode.opModeIsActive()) {
            // Check timeout
            if (System.currentTimeMillis() - startTime > AlignmentParams.ALIGNMENT_TIMEOUT_MS) {
                opMode.telemetry.addLine("⚠️ Alignment timeout");
                opMode.telemetry.update();
                break;
            }

            // Get current TX reading
            if (!findTarget()) {
                opMode.telemetry.addLine("⚠️ Lost target");
                opMode.telemetry.update();
                currentState = AlignmentState.TARGET_LOST;
                break;
            }


            // Check if aligned
            if (Math.abs(currentTx) <= AlignmentParams.TX_TOLERANCE_DEGREES) {
                stableFrames++;
                opMode.telemetry.addLine("✓ Within tolerance");

                if (stableFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                    opMode.telemetry.addLine("✓✓✓ ALIGNED! ✓✓✓");
                    opMode.telemetry.update();
                    turretController.stop();
                    sleep((long)AlignmentParams.SETTLING_TIME_MS);
                    break;
                }

                // Stop while counting stable frames
                turretController.stop();
            } else {
                // Reset stable counter if we drift
                stableFrames = 0;

                // Determine turn direction and apply power
                double power = (currentTx > 0) ? -AlignmentParams.TURN_POWER : AlignmentParams.TURN_POWER;
                turretController.setPower(power);

                opMode.telemetry.addData("Power", "%.2f", power);
            }

            opMode.telemetry.update();

            // Loop timing (50Hz)
            sleep(20);
        }

        // Ensure stopped
        turretController.stop();
    }

    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        turretController.stop();
    }

    public void maintainAlignment() {
        // Continuously check and adjust if needed
        if (!findTarget()) return;

        if (Math.abs(currentTx) > AlignmentParams.TX_TOLERANCE_DEGREES) {
            double power = (currentTx > 0) ? AlignmentParams.TURN_POWER : -AlignmentParams.TURN_POWER;
            turretController.setPower(power);
        } else {
            turretController.stop();
        }
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
                    sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return false;
                }
            }

            if (validReadings > 0) {
                currentTx = sumTx / validReadings;
                currentTx = -currentTx;  // Invert TX sign
                hasTarget = true;
                return true;
            }

        } catch (Exception ignored) {
        }

        hasTarget = false;
        return false;
    }

    private void executeSearch() {
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
            hasTarget = true;

            try {
                turnTurretToAlignWithTarget();
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentState = AlignmentState.STOPPED;
            }
            return;
        }

        // Continue searching - slow rotation
        turretController.setPower(TargetSearch.SEARCH_POWER);
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
        return Math.abs(currentTx);
    }

    public double getTotalAlignmentTime() {
        return totalAlignmentTime;
    }

    public double getCurrentTurretAngle() {
        return turretController.getCurrentAngle();
    }

    public double getCurrentTx() {
        return currentTx;
    }
}