package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import java.util.List;

/**
 * PID-based turret alignment using Limelight AprilTag detection
 * Aligns turret to center on AprilTag 20 within 5 degrees tolerance
 */
@Config
public class TurretAlignmentController {

    private final LinearOpMode opMode;
    private final TurretController turretController;
    private Limelight3A limelight;

    // === TUNABLE PARAMETERS ===

    @Config
    public static class AlignmentParams {
        public static double TARGET_TOLERANCE_DEGREES = 5.0;
        public static long ALIGNMENT_TIMEOUT_MS = 3000;
        public static int ALIGNED_FRAMES_REQUIRED = 3;
        public static double SETTLING_TIME_MS = 100;
    }

    @Config
    public static class PIDParams {
        // PID gains - tune these for your turret
        public static double KP = 0.015;  // Proportional gain
        public static double KI = 0.0;    // Integral gain
        public static double KD = 0.002;  // Derivative gain

        // Output limits (servo power)
        public static double MAX_POWER = 0.8;
        public static double MIN_POWER = 0.15;

        // Anti-windup
        public static double INTEGRAL_MAX = 50.0;

        // Deadband
        public static double DEADBAND_DEGREES = 1.0;
    }

    @Config
    public static class SearchParams {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_POWER = 0.3;
        public static long MAX_SEARCH_TIME_MS = 5000;
    }

    @Config
    public static class LimelightParams {
        public static int APRILTAG_PIPELINE = 1;
        public static int DETECTION_SAMPLES = 5;
    }

    // === PID CONTROLLER ===

    private static class PIDController {
        private double lastError = 0;
        private double integral = 0;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean firstUpdate = true;

        public void reset() {
            lastError = 0;
            integral = 0;
            firstUpdate = true;
            timer.reset();
        }

        public double calculate(double error) {
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
            output = Math.max(-PIDParams.MAX_POWER,
                    Math.min(PIDParams.MAX_POWER, output));

            // Apply minimum power if we're still moving
            if (Math.abs(output) > 0 && Math.abs(output) < PIDParams.MIN_POWER) {
                output = Math.copySign(PIDParams.MIN_POWER, output);
            }

            // Store for next iteration
            lastError = error;

            return output;
        }
    }

    // === STATE VARIABLES ===

    public enum AlignmentState {
        IDLE, SEARCHING, ALIGNING, ALIGNED, TARGET_LOST, ERROR
    }

    private AlignmentState currentState = AlignmentState.IDLE;
    private final PIDController pidController = new PIDController();

    private int targetTagId = 20;
    private boolean hasTarget = false;
    private double currentTx = 0;
    private int consecutiveAlignedFrames = 0;

    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private final ElapsedTime searchTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // === INITIALIZATION ===

    public TurretAlignmentController(LinearOpMode opMode, TurretController turretController) {
        this.opMode = opMode;
        this.turretController = turretController;

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.pipelineSwitch(LimelightParams.APRILTAG_PIPELINE);  // Use configurable AprilTag pipeline
            this.limelight.start();
            opMode.telemetry.addLine("✓ Limelight initialized for turret alignment");
            opMode.telemetry.addData("Pipeline", LimelightParams.APRILTAG_PIPELINE);
        } catch (Exception e) {
            this.limelight = null;
            opMode.telemetry.addLine("⚠️ Limelight not found - turret alignment disabled");
            opMode.telemetry.addData("  Error", e.getMessage());
        }
    }

    // === PUBLIC API ===

    /**
     * Start alignment process
     */
    public void startAlignment() {
        if (limelight == null) {
            currentState = AlignmentState.ERROR;
            return;
        }

        currentState = AlignmentState.SEARCHING;
        pidController.reset();
        consecutiveAlignedFrames = 0;
        alignmentTimer.reset();
        searchTimer.reset();

        opMode.telemetry.addLine("=== TURRET ALIGNMENT STARTED ===");
        opMode.telemetry.addData("Target Tag", targetTagId);
        opMode.telemetry.update();
    }

    /**
     * Main update loop - call this repeatedly
     */
    public void update() {
        if (limelight == null) {
            currentState = AlignmentState.ERROR;
            turretController.setPower(0);
            return;
        }

        switch (currentState) {
            case SEARCHING:
                executeSearch();
                break;

            case ALIGNING:
                executeAlignment();
                break;

            case ALIGNED:
                maintainAlignment();
                break;

            case TARGET_LOST:
            case IDLE:
            case ERROR:
                turretController.setPower(0);
                break;
        }

        // Update turret controller
        turretController.update();
    }

    /**
     * Stop alignment and cancel any active movements
     */
    public void stopAlignment() {
        currentState = AlignmentState.IDLE;
        turretController.setPower(0);
        pidController.reset();

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }
    }

    /**
     * Set target AprilTag ID
     */
    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    // === CORE ALIGNMENT LOGIC ===

    /**
     * Search for the target AprilTag
     */
    private void executeSearch() {
        if (!SearchParams.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.setPower(0);
            return;
        }

        // Check timeout
        if (searchTimer.milliseconds() > SearchParams.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.setPower(0);

            opMode.telemetry.addLine("⚠️ Search timeout - target not found");
            opMode.telemetry.addData("Checked Pipeline", LimelightParams.APRILTAG_PIPELINE);
            opMode.telemetry.addData("Detection Samples", LimelightParams.DETECTION_SAMPLES);
            opMode.telemetry.update();
            return;
        }

        // Try to find target
        if (scanForTarget()) {
            currentState = AlignmentState.ALIGNING;
            pidController.reset();

            opMode.telemetry.addLine("✓ Target found! Starting alignment...");
            opMode.telemetry.addData("Initial TX Error", "%.2f°", currentTx);
            opMode.telemetry.update();
            return;
        }

        // Continue searching - rotate turret slowly
        turretController.setPower(SearchParams.SEARCH_POWER);
    }

    /**
     * Execute PID-controlled alignment
     */
    private void executeAlignment() {
        // Check timeout
        if (alignmentTimer.milliseconds() > AlignmentParams.ALIGNMENT_TIMEOUT_MS) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.setPower(0);

            opMode.telemetry.addLine("⚠️ Alignment timeout");
            opMode.telemetry.update();
            return;
        }

        // Scan for target and get current error
        if (!scanForTarget()) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.setPower(0);
            return;
        }

        // Calculate PID output
        double power = pidController.calculate(currentTx);

        // Check if aligned
        if (Math.abs(currentTx) <= AlignmentParams.TARGET_TOLERANCE_DEGREES) {
            consecutiveAlignedFrames++;

            if (consecutiveAlignedFrames >= AlignmentParams.ALIGNED_FRAMES_REQUIRED) {
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
                turretController.setPower(0);

                opMode.telemetry.addLine("✓✓✓ TURRET ALIGNED! ✓✓✓");
                opMode.telemetry.addData("Final Error", "%.2f°", currentTx);
                opMode.telemetry.addData("Time", "%.2f s", totalAlignmentTime);
                opMode.telemetry.update();

                try {
                    Thread.sleep((long)AlignmentParams.SETTLING_TIME_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                return;
            }
        } else {
            consecutiveAlignedFrames = 0;
        }

        // Apply PID power to turret
        turretController.setPower(power);

        // Display telemetry
        opMode.telemetry.addLine("=== TURRET ALIGNING ===");
        opMode.telemetry.addData("TX Error", "%.2f°", currentTx);
        opMode.telemetry.addData("PID Power", "%.3f", power);
        opMode.telemetry.addData("Turret Pos", "%.1f°", turretController.getCurrentPosition());
        opMode.telemetry.addData("Stable Frames", "%d/%d",
                consecutiveAlignedFrames, AlignmentParams.ALIGNED_FRAMES_REQUIRED);
        opMode.telemetry.addData("Time", "%.2f s", alignmentTimer.seconds());
        opMode.telemetry.update();
    }

    /**
     * Maintain alignment while already aligned
     */
    private void maintainAlignment() {
        // Continuously check target and apply corrections
        if (!scanForTarget()) {
            currentState = AlignmentState.TARGET_LOST;
            turretController.setPower(0);
            return;
        }

        // If we drift out of tolerance, go back to aligning
        if (Math.abs(currentTx) > AlignmentParams.TARGET_TOLERANCE_DEGREES * 1.5) {
            currentState = AlignmentState.ALIGNING;
            pidController.reset();
            consecutiveAlignedFrames = 0;
            return;
        }

        // Apply small corrections
        double power = pidController.calculate(currentTx);
        turretController.setPower(power);
    }

    /**
     * Scan Limelight for target AprilTag and update TX error
     * @return true if target found, false otherwise
     */
    private boolean scanForTarget() {
        if (limelight == null) {
            return false;
        }

        try {
            // Take multiple readings for better accuracy
            double sumTx = 0;
            int validReadings = 0;

            for (int i = 0; i < LimelightParams.DETECTION_SAMPLES; i++) {
                LLResult result = limelight.getLatestResult();

                // Debug: Show if result is valid and number of fiducials detected
                opMode.telemetry.addData("Limelight Valid", result != null && result.isValid());
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    opMode.telemetry.addData("Fiducials Count", fiducials.size());

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        opMode.telemetry.addData("Fiducial ID", fiducial.getFiducialId());
                        opMode.telemetry.addData("TX Degrees", fiducial.getTargetXDegrees());

                        if (fiducial.getFiducialId() == targetTagId) {
                            sumTx += fiducial.getTargetXDegrees();
                            validReadings++;
                            break; // Found target, no need to check other fiducials
                        }
                    }
                } else {
                    opMode.telemetry.addData("Result Valid", result != null ? "Null result" : "Invalid result");
                }

                // Small delay between samples
                try {
                    opMode.sleep(10);
                } catch (Exception e) {
                    // Ignore sleep interruption
                }
            }

            if (validReadings > 0) {
                // Average the TX readings and invert sign (negative TX means target is to the left)
                currentTx = -(sumTx / validReadings);
                hasTarget = true;
                return true;
            }

        } catch (Exception e) {
            opMode.telemetry.addLine("⚠️ Limelight error: " + e.getMessage());
        }

        hasTarget = false;
        return false;
    }

    // === TELEMETRY ===

    public void displayTelemetry() {
        opMode.telemetry.addLine("═══════════════════════");
        opMode.telemetry.addLine("  TURRET ALIGNMENT");
        opMode.telemetry.addLine("═══════════════════════");
        opMode.telemetry.addLine();

        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target Tag", targetTagId);
        opMode.telemetry.addData("Has Target", hasTarget);
        opMode.telemetry.addLine();

        if (hasTarget) {
            opMode.telemetry.addData("TX Error", "%.2f°", currentTx);
            opMode.telemetry.addData("Within Tolerance",
                    Math.abs(currentTx) <= AlignmentParams.TARGET_TOLERANCE_DEGREES);
        }

        opMode.telemetry.addLine();
        opMode.telemetry.addData("Turret Position", "%.1f°",
                turretController.getCurrentPosition());
        opMode.telemetry.addData("Turret Velocity", "%.1f°/s",
                turretController.getVelocity());

        if (totalAlignmentTime > 0) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Last Alignment Time", "%.2f s", totalAlignmentTime);
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

    public boolean isAligning() {
        return currentState == AlignmentState.ALIGNING;
    }

    public AlignmentState getState() {
        return currentState;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getCurrentError() {
        return currentTx;
    }

    public double getTotalAlignmentTime() {
        return totalAlignmentTime;
    }

    public double getCurrentTurretPosition() {
        return turretController.getCurrentPosition();
    }

    public boolean isLimelightAvailable() {
        return limelight != null;
    }
}