package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;
import java.util.List;

/**
 * Fixed velocity-controlled AprilTag alignment with LOCKED PID zones
 * PID values are determined at START and DO NOT change during alignment
 */
@Config
public class LimelightAlignmentController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    // === TUNABLE PARAMETERS ===

    @Config
    public static class PIDZones {
        // Zone 1: Fine alignment (0-5°)
        public static double ZONE1_MAX = 5.0;
        public static double ZONE1_KP = 1.5;
        public static double ZONE1_KD = 0;

        // Zone 2: Medium error (5-15°)
        public static double ZONE2_MAX = 15.0;
        public static double ZONE2_KP = 1;
        public static double ZONE2_KD = 0;

        // Zone 3: Large error (>15°)
        public static double ZONE3_KP = 1;
        public static double ZONE3_KD = 0;

        // Common I term (usually keep at 0)
        public static double KI = 0;
        public static double INTEGRAL_LIMIT = 3.0;
    }

    @Config
    public static class VelocityLimits {
        public static double MAX_ANGULAR_VELOCITY = 160.0;  // degrees/sec
        public static double MIN_ANGULAR_VELOCITY = 15.0;   // degrees/sec
        public static double MAX_ACCELERATION = 100;       // degrees/sec²
    }

    @Config
    public static class AlignmentThresholds {
        public static double TOLERANCE = 0.5;              // Aligned when within this
        public static double DEAD_ZONE = 2;              // Start slowing down here
        public static int ALIGNED_FRAMES_REQUIRED = 15;    // Frames to confirm alignment
    }

    @Config
    public static class Filtering {
        public static double INPUT_FILTER = 0.7;   // Limelight tx filtering
        public static double OUTPUT_FILTER = 0.8;  // Velocity command filtering
    }

    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_VELOCITY = 150.0;       // degrees/sec
        public static double SEARCH_DELAY_MS = 300;
        public static double MAX_SEARCH_TIME_MS = 5000;
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
    private double tx = 0;
    private double filteredTx = 0;

    // LOCKED PID GAINS - Set once at the start, never changed
    private double lockedKP = 0;
    private double lockedKD = 0;
    private boolean pidGainsLocked = false;

    // PID state
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // Velocity control
    private double targetAngularVelocity = 0;
    private double currentAngularVelocity = 0;
    private double lastAngularVelocity = 0;
    private final ElapsedTime accelerationTimer = new ElapsedTime();

    // Alignment tracking
    private int consecutiveAlignedFrames = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();
    private double totalAlignmentTime = 0;

    // Search state
    private final ElapsedTime searchTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Initial conditions (for telemetry)
    private double initialAngle = 0;
    private boolean initialAngleCaptured = false;
    private String currentZone = "N/A";

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

        pidTimer.reset();
        accelerationTimer.reset();
    }

    // === PUBLIC API ===

    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    public void startAlignment() {
        isActive = true;
        initialAngleCaptured = false;
        initialAngle = 0;
        pidGainsLocked = false;  // Reset PID lock
        alignmentTimer.reset();

        updateTracking();

        if (hasTarget) {
            captureInitialAngle();
            lockPIDGains();  // Lock PID gains based on initial error
            currentState = AlignmentState.ALIGNING;
        } else {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
        }

        consecutiveAlignedFrames = 0;
        resetPID();
    }

    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;

        if (alignmentTimer.seconds() > 0) {
            totalAlignmentTime = alignmentTimer.seconds();
        }

        smoothStop();
        resetPID();
        pidGainsLocked = false;  // Unlock for next alignment
    }

    public void align(int targetTagId) {
        if (!isActive) return;

        this.targetTagId = targetTagId;
        updateTracking();

        if (hasTarget && !initialAngleCaptured) {
            captureInitialAngle();
        }

        // Lock PID gains if not already locked and we have a target
        if (hasTarget && !pidGainsLocked) {
            lockPIDGains();
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
                handleTargetLost();
                break;
            case STOPPED:
                break;
        }

        sendDashboardData();
    }

    // === CORE ALIGNMENT LOGIC ===

    private void executeSearch() {
        if (!TargetSearch.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            smoothStop();
            return;
        }

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            searchTimer.reset();
            lockPIDGains();  // Lock gains when target found
            resetPID();
            return;
        }

        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            smoothStop();
            return;
        }

        // Constant velocity search
        applyAngularVelocity(TargetSearch.SEARCH_VELOCITY);
    }

    private void executeAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            return;
        }

        double error = filteredTx;
        double absError = Math.abs(error);

        // Check if aligned
        if (absError <= AlignmentThresholds.TOLERANCE) {
            consecutiveAlignedFrames++;
            if (consecutiveAlignedFrames >= AlignmentThresholds.ALIGNED_FRAMES_REQUIRED) {
                currentState = AlignmentState.ALIGNED;
                totalAlignmentTime = alignmentTimer.seconds();
                smoothStop();
                return;
            }
        } else {
            consecutiveAlignedFrames = Math.max(0, consecutiveAlignedFrames - 2);
        }

        // Calculate desired velocity using LOCKED PID gains
        double desiredVelocity = calculatePIDWithLockedGains(error);

        // Progressive power reduction near target
        if (absError < AlignmentThresholds.DEAD_ZONE) {
            double scale = absError / AlignmentThresholds.DEAD_ZONE;
            scale = Math.pow(scale, 1.5);  // Smoother approach
            desiredVelocity *= scale;
        }

        // Apply acceleration limiting
        desiredVelocity = applyAccelerationLimit(desiredVelocity);

        // Output filtering for smoothness
        targetAngularVelocity = (Filtering.OUTPUT_FILTER * desiredVelocity) +
                ((1 - Filtering.OUTPUT_FILTER) * lastAngularVelocity);
        lastAngularVelocity = targetAngularVelocity;

        // Command the velocity
        applyAngularVelocity(targetAngularVelocity);
    }

    /**
     * Lock PID gains based on INITIAL error - these gains will NOT change during alignment
     */
    private void lockPIDGains() {
        if (pidGainsLocked || !hasTarget) return;

        double absError = Math.abs(filteredTx);

        // Determine which zone we're in and lock those gains
        if (absError <= PIDZones.ZONE1_MAX) {
            lockedKP = PIDZones.ZONE1_KP;
            lockedKD = PIDZones.ZONE1_KD;
            currentZone = String.format("Zone 1 (0-%.1f°) LOCKED", PIDZones.ZONE1_MAX);
        } else if (absError <= PIDZones.ZONE2_MAX) {
            lockedKP = PIDZones.ZONE2_KP;
            lockedKD = PIDZones.ZONE2_KD;
            currentZone = String.format("Zone 2 (%.1f-%.1f°) LOCKED",
                    PIDZones.ZONE1_MAX, PIDZones.ZONE2_MAX);
        } else {
            lockedKP = PIDZones.ZONE3_KP;
            lockedKD = PIDZones.ZONE3_KD;
            currentZone = String.format("Zone 3 (>%.1f°) LOCKED", PIDZones.ZONE2_MAX);
        }

        pidGainsLocked = true;

        // Log the locked gains
        opMode.telemetry.addLine("=== PID GAINS LOCKED ===");
        opMode.telemetry.addData("Initial Error", "%.2f°", absError);
        opMode.telemetry.addData("Locked Zone", currentZone);
        opMode.telemetry.addData("Locked KP", "%.2f", lockedKP);
        opMode.telemetry.addData("Locked KD", "%.2f", lockedKD);
        opMode.telemetry.update();
    }

    /**
     * Calculate PID output using LOCKED gains that never change
     */
    private double calculatePIDWithLockedGains(double error) {
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;

        if (dt <= 0 || lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return lockedKP * error;
        }

        lastTime = currentTime;
        double absError = Math.abs(error);

        // Use LOCKED gains throughout
        double kp = lockedKP;
        double kd = lockedKD;

        // P term
        double pTerm = kp * error;

        // I term with anti-windup
        if (Math.signum(error) != Math.signum(lastError) &&
                absError > AlignmentThresholds.TOLERANCE) {
            integralSum *= 0.5;  // Reset on sign change
        } else {
            integralSum += error * dt;
        }
        integralSum = Range.clip(integralSum,
                -PIDZones.INTEGRAL_LIMIT,
                PIDZones.INTEGRAL_LIMIT);
        double iTerm = PIDZones.KI * integralSum;

        // D term
        double derivative = (error - lastError) / dt;
        derivative = Range.clip(derivative, -100, 100);
        double dTerm = kd * derivative;

        lastError = error;

        // Combine PID terms
        double output = pTerm + iTerm + dTerm;
        output = Range.clip(output,
                -VelocityLimits.MAX_ANGULAR_VELOCITY,
                VelocityLimits.MAX_ANGULAR_VELOCITY);

        // Minimum velocity threshold
        if (absError > AlignmentThresholds.DEAD_ZONE &&
                Math.abs(output) > 0 &&
                Math.abs(output) < VelocityLimits.MIN_ANGULAR_VELOCITY) {
            output = Math.signum(output) * VelocityLimits.MIN_ANGULAR_VELOCITY;
        }

        return output;
    }

    private double applyAccelerationLimit(double targetVelocity) {
        double dt = accelerationTimer.seconds();
        accelerationTimer.reset();

        if (dt <= 0 || dt > 0.5) return targetVelocity;

        double maxChange = VelocityLimits.MAX_ACCELERATION * dt;
        double velocityDiff = targetVelocity - currentAngularVelocity;

        if (Math.abs(velocityDiff) > maxChange) {
            return currentAngularVelocity + Math.signum(velocityDiff) * maxChange;
        }

        return targetVelocity;
    }

    private void applyAngularVelocity(double degreesPerSecond) {
        currentAngularVelocity = degreesPerSecond;
        // Negative because counterclockwise is positive in convention
        driveController.setAngularVelocity(-degreesPerSecond);
    }

    private void maintainAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
            return;
        }

        double absError = Math.abs(filteredTx);

        // If error grows, go back to aligning
        if (absError > AlignmentThresholds.DEAD_ZONE) {
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            return;
        }

        // Small correction if needed using locked gains
        if (absError > AlignmentThresholds.TOLERANCE) {
            double correctionVel = lockedKP * filteredTx * 0.3;
            correctionVel = Range.clip(correctionVel,
                    -VelocityLimits.MIN_ANGULAR_VELOCITY,
                    VelocityLimits.MIN_ANGULAR_VELOCITY);
            applyAngularVelocity(correctionVel);
        } else {
            smoothStop();
        }
    }

    private void handleTargetLost() {
        if (TargetSearch.ENABLE_SEARCH &&
                stateTimer.milliseconds() > TargetSearch.SEARCH_DELAY_MS) {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
            pidGainsLocked = false;  // Unlock for new search
            return;
        }

        smoothStop();
        consecutiveAlignedFrames = 0;

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            if (!pidGainsLocked) {
                lockPIDGains();  // Lock gains if we found target again
            }
            integralSum *= 0.5;
        }
    }

    // === HELPER METHODS ===

    private void updateTracking() {
        if (limelight == null) {
            hasTarget = false;
            return;
        }

        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                boolean previousHasTarget = hasTarget;
                hasTarget = false;

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetTagId) {
                        hasTarget = true;
                        tx = fiducial.getTargetXDegrees();

                        // Input filtering
                        if (!previousHasTarget) {
                            filteredTx = tx;
                        } else {
                            filteredTx = (Filtering.INPUT_FILTER * tx) +
                                    ((1 - Filtering.INPUT_FILTER) * filteredTx);
                        }
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

    public void displayAlignmentWithInitialAngle() {
        opMode.telemetry.addLine("╔═══════════════════════════╗");
        opMode.telemetry.addLine("║  ALIGNMENT STATUS         ║");
        opMode.telemetry.addLine("╚═══════════════════════════╝");

        // Show initial angle prominently
        if (initialAngleCaptured) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine("┌─── INITIAL CONDITIONS ───┐");
            opMode.telemetry.addData("│ Starting Angle", String.format(Locale.US, "%.2f°", initialAngle));
            opMode.telemetry.addLine("└─────────────────────────┘");
        }

        opMode.telemetry.addLine();
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target", hasTarget ? "✓ FOUND" : "✗ LOST");
        opMode.telemetry.addData("Zone", currentZone);

        if (hasTarget) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine("─── CURRENT ERROR ───");
            opMode.telemetry.addData("TX Angle", String.format(Locale.US, "%.2f°", filteredTx));
            opMode.telemetry.addData("Error", String.format(Locale.US, "%.2f°", Math.abs(filteredTx)));

            // Show progress if we have initial angle
            if (initialAngleCaptured && initialAngle > 0) {
                double improvement = ((initialAngle - Math.abs(filteredTx)) / initialAngle) * 100;
                improvement = Math.max(0, Math.min(100, improvement)); // Clamp to 0-100%
                opMode.telemetry.addData("Progress", String.format(Locale.US, "%.0f%%", improvement));
            }
        }

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("─── VELOCITY ───");
        opMode.telemetry.addData("Target", String.format(Locale.US, "%.1f °/s", targetAngularVelocity));
        opMode.telemetry.addData("Current", String.format(Locale.US, "%.1f °/s", currentAngularVelocity));

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("─── LOCKED PID GAINS ───");
        opMode.telemetry.addData("KP", String.format(Locale.US, "%.2f", lockedKP));
        opMode.telemetry.addData("KI", String.format(Locale.US, "%.3f", PIDZones.KI));
        opMode.telemetry.addData("KD", String.format(Locale.US, "%.2f", lockedKD));
        opMode.telemetry.addData("Locked", pidGainsLocked ? "YES" : "NO");

        if (currentState == AlignmentState.ALIGNED && totalAlignmentTime > 0) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine("╔═══════════════════════════╗");
            opMode.telemetry.addData("║ ✓ ALIGNED", String.format(Locale.US, "%.2fs", totalAlignmentTime));
            if (initialAngleCaptured) {
                opMode.telemetry.addData("║ From", String.format(Locale.US, "%.2f° → 0°", initialAngle));
            }
            opMode.telemetry.addLine("╚═══════════════════════════╝");
        }

        opMode.telemetry.update();
    }

    private void captureInitialAngle() {
        if (!initialAngleCaptured && hasTarget) {
            initialAngle = Math.abs(filteredTx);
            initialAngleCaptured = true;
        }
    }

    private void smoothStop() {
        double stoppingVelocity = currentAngularVelocity * 0.5;

        if (Math.abs(stoppingVelocity) < VelocityLimits.MIN_ANGULAR_VELOCITY) {
            currentAngularVelocity = 0;
            targetAngularVelocity = 0;
            driveController.stopDrive();
        } else {
            applyAngularVelocity(stoppingVelocity);
        }
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        targetAngularVelocity = 0;
        currentAngularVelocity = 0;
        lastAngularVelocity = 0;
        pidTimer.reset();
        accelerationTimer.reset();
    }

    // === TELEMETRY ===

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // State
        packet.put("State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("Current_Zone", currentZone);
        packet.put("PID_Locked", pidGainsLocked);

        // Error tracking
        packet.put("TX_Raw", tx);
        packet.put("TX_Filtered", filteredTx);
        packet.put("Error", Math.abs(filteredTx));

        // Velocity
        packet.put("Target_Velocity", targetAngularVelocity);
        packet.put("Current_Velocity", currentAngularVelocity);
        packet.put("Left_Wheel_Vel", driveController.getLeftVelocity());
        packet.put("Right_Wheel_Vel", driveController.getRightVelocity());

        // LOCKED PID
        packet.put("Locked_KP", lockedKP);
        packet.put("Locked_KD", lockedKD);
        packet.put("KI", PIDZones.KI);
        packet.put("Integral_Sum", integralSum);

        // Timing
        packet.put("Aligned_Frames", consecutiveAlignedFrames);
        packet.put("Alignment_Time", alignmentTimer.seconds());
        if (totalAlignmentTime > 0) {
            packet.put("Total_Time", totalAlignmentTime);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayTelemetry() {
        opMode.telemetry.addLine("═══ ALIGNMENT STATUS ═══");
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target", hasTarget ? "FOUND" : "LOST");
        opMode.telemetry.addData("Current Zone", currentZone);
        opMode.telemetry.addData("PID Locked", pidGainsLocked ? "YES" : "NO");

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("═══ ERROR ═══");
        if (hasTarget) {
            opMode.telemetry.addData("TX Angle", "%.2f°", filteredTx);
            opMode.telemetry.addData("Error", "%.2f°", Math.abs(filteredTx));
            if (initialAngleCaptured) {
                opMode.telemetry.addData("Initial Angle", "%.2f°", initialAngle);
            }
        }

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("═══ VELOCITY ═══");
        opMode.telemetry.addData("Target", "%.1f °/s", targetAngularVelocity);
        opMode.telemetry.addData("Current", "%.1f °/s", currentAngularVelocity);
        opMode.telemetry.addData("Left Wheel", "%.0f ticks/s", driveController.getLeftVelocity());
        opMode.telemetry.addData("Right Wheel", "%.0f ticks/s", driveController.getRightVelocity());

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("═══ LOCKED PID GAINS ═══");
        opMode.telemetry.addData("KP", "%.2f", lockedKP);
        opMode.telemetry.addData("KI", "%.3f", PIDZones.KI);
        opMode.telemetry.addData("KD", "%.2f", lockedKD);

        if (currentState == AlignmentState.ALIGNED && totalAlignmentTime > 0) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("✓ Aligned in", "%.2fs", totalAlignmentTime);
        }

        opMode.telemetry.update();
    }

    // === GETTERS ===

    public boolean isAligned() { return currentState == AlignmentState.ALIGNED; }
    public boolean isSearching() { return currentState == AlignmentState.SEARCHING; }
    public AlignmentState getState() { return currentState; }
    public boolean hasTarget() { return hasTarget; }
    public double getTargetError() { return Math.abs(filteredTx); }
    public double getTotalAlignmentTime() { return totalAlignmentTime; }
    public String getCurrentZone() { return currentZone; }
    public double getLockedKP() { return lockedKP; }
    public double getLockedKD() { return lockedKD; }
}