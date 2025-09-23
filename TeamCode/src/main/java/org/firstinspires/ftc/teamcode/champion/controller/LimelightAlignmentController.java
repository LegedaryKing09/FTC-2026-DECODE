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

import java.util.List;

/**
 * Smooth AprilTag alignment controller with target search - Enhanced version
 * Uses exact PID values from working LimelightNotJerky code
 * Added: Automatic right turn search when target is lost during alignment
 */
@Config
public class LimelightAlignmentController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    // IMPORTANT: These Config classes must have unique names to avoid conflicts!
    @Config
    public static class AlignmentPID {
        // EXACT VALUES FROM YOUR WORKING CODE!
        public static double KP = 0.04;
        public static double KI = 0.0003;  // Changed from 0.0005
        public static double KD = 0.02;    // Changed from 0.008
        public static double INTEGRAL_LIMIT = 3.0;
    }

    @Config
    public static class AlignmentZones {
        public static double TOLERANCE = 0.5;      // Perfect alignment
        public static double DEAD_ZONE = 2.0;      // Good enough
    }

    @Config
    public static class AlignmentSpeed {
        public static double MAX_SPEED = 0.5;
        public static double MIN_SPEED = 0.0;      // CRITICAL! Changed from 0.3 to 0.0
        public static double MAX_ACCELERATION = 1.5;  // Changed from 2.0
    }

    @Config
    public static class AlignmentFiltering {
        public static double INPUT_FILTER = 0.7;
        public static double OUTPUT_FILTER = 0.8;
    }

    @Config
    public static class AlignmentAdvanced {
        public static boolean ADAPTIVE_GAIN = true;
        public static double ADAPTIVE_FACTOR = 0.8;
        public static int ALIGNED_FRAMES = 15;
        public static double PROGRESSIVE_POWER = 1.5;
    }

    // NEW: Target search configuration
    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;        // Enable/disable search feature
        public static double SEARCH_SPEED = 0.25;          // Speed for searching (right turn)
        public static double SEARCH_DELAY_MS = 500;        // Wait time before starting search
        public static double MAX_SEARCH_TIME_MS = 5000;    // Maximum search time before giving up
    }

    // Tracking state
    private double tx = 0;
    private double filteredTx = 0;
    private boolean hasTarget = false;
    private int targetTagId = 20;  // Default, can be changed

    // PID state
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // Smoothing state
    private double lastOutput = 0;
    private double currentOutput = 0;
    private final ElapsedTime accelerationTimer = new ElapsedTime();

    // State tracking - ENHANCED with search state
    public enum AlignmentState {
        ALIGNING, ALIGNED, TARGET_LOST, SEARCHING, STOPPED
    }
    private AlignmentState currentState = AlignmentState.STOPPED;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int consecutiveAlignedFrames = 0;

    // NEW: Search state tracking
    private final ElapsedTime searchTimer = new ElapsedTime();
    private boolean wasAligning = false;  // Track if we were previously aligning

    // Performance metrics
    private double maxError = 0;
    private double minError = Double.MAX_VALUE;
    private double avgError = 0;
    private int errorSamples = 0;

    // Control flags
    private boolean isActive = false;

    /**
     * Constructor
     */
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
            this.limelight.pipelineSwitch(0);
            this.limelight.start();
        } catch (Exception e) {
            throw new Exception("Failed to initialize Limelight: " + e.getMessage());
        }

        pidTimer.reset();
        accelerationTimer.reset();
        stateTimer.reset();
        searchTimer.reset();
    }

    /**
     * Set the target AprilTag ID
     */
    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    /**
     * Start alignment process
     */
    public void startAlignment() {
        isActive = true;
        currentState = AlignmentState.TARGET_LOST;
        consecutiveAlignedFrames = 0;
        wasAligning = false;
        resetPID();
        resetSearchState();
    }

    /**
     * Stop alignment and return control to manual driving
     */
    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;
        driveController.stopDrive();
        resetPID();
        resetSearchState();
    }

    /**
     * Main alignment update - call this continuously
     */
    public void align(int targetTagId) {
        if (!isActive) return;

        this.targetTagId = targetTagId;
        updateTracking();
        updateState();

        switch (currentState) {
            case ALIGNING:
                executeSmoothedAlignment();
                break;
            case ALIGNED:
                maintainAlignment();
                break;
            case TARGET_LOST:
                handleTargetLost();
                break;
            case SEARCHING:
                executeTargetSearch();
                break;
            case STOPPED:
                // Do nothing
                break;
        }

        updateMetrics();
        sendDashboardData();
    }

    /**
     * Check if currently aligned
     */
    public boolean isAligned() {
        return currentState != AlignmentState.ALIGNED;
    }

    /**
     * Check if currently searching
     */
    public boolean isSearching() {
        return currentState == AlignmentState.SEARCHING;
    }

    /**
     * Get current state
     */
    public AlignmentState getState() {
        return currentState;
    }

    /**
     * Get target status
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    /**
     * Get current error
     */
    public double getTargetError() {
        return Math.abs(filteredTx);
    }

    /**
     * NEW: Execute target search by turning right
     */
    private void executeTargetSearch() {
        if (!TargetSearch.ENABLE_SEARCH) {
            // If search is disabled, just handle as normal target lost
            handleTargetLostNoSearch();
            return;
        }

        // Check if we found the target
        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            resetSearchState();
            return;
        }

        // Check search timeout
        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            resetSearchState();
            smoothStop();
            return;
        }

        // Execute search turn (right turn = positive power for left wheels, negative for right)
        double searchPower = TargetSearch.SEARCH_SPEED;
        driveController.tankDrive(searchPower, -searchPower);  // Turn right

        // Update tracking for smooth output
        currentOutput = searchPower;
        lastOutput = currentOutput;
    }

    /**
     * Main alignment execution (unchanged from working version)
     */
    private void executeSmoothedAlignment() {
        if (!hasTarget) {
            smoothStop();
            return;
        }

        double headingError = filteredTx;
        double absError = Math.abs(headingError);

        // Check alignment
        if (absError <= AlignmentZones.TOLERANCE) {
            consecutiveAlignedFrames++;
            if (consecutiveAlignedFrames >= AlignmentAdvanced.ALIGNED_FRAMES) {
                currentState = AlignmentState.ALIGNED;
                stateTimer.reset();
                smoothStop();
                return;
            }
        } else {
            consecutiveAlignedFrames = Math.max(0, consecutiveAlignedFrames - 2);
        }

        // Calculate PID output
        double turnPower = calculateSmoothedPID(headingError);

        // Progressive scaling near target
        if (absError < AlignmentZones.DEAD_ZONE) {
            double scale = absError / AlignmentZones.DEAD_ZONE;
            scale = Math.pow(scale, AlignmentAdvanced.PROGRESSIVE_POWER);
            turnPower *= scale;
        }

        // Apply acceleration limiting
        turnPower = applyAccelerationLimit(turnPower);

        // Apply output filtering
        currentOutput = (AlignmentFiltering.OUTPUT_FILTER * turnPower) +
                ((1 - AlignmentFiltering.OUTPUT_FILTER) * lastOutput);
        lastOutput = currentOutput;

        applyTurn(currentOutput);
    }

    /**
     * Calculate PID with exact logic from working code (unchanged)
     */
    private double calculateSmoothedPID(double error) {
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;

        if (dt <= 0 || lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return AlignmentPID.KP * error;
        }

        lastTime = currentTime;

        // Adaptive gain
        double gainMultiplier = 1.0;
        if (AlignmentAdvanced.ADAPTIVE_GAIN) {
            double absError = Math.abs(error);
            if (absError < AlignmentZones.DEAD_ZONE * 2) {
                gainMultiplier = AlignmentAdvanced.ADAPTIVE_FACTOR +
                        (1 - AlignmentAdvanced.ADAPTIVE_FACTOR) * (absError / (AlignmentZones.DEAD_ZONE * 2));
            }
        }

        // P term
        double pTerm = AlignmentPID.KP * error * gainMultiplier;

        // I term with anti-oscillation
        if (Math.signum(error) != Math.signum(lastError) && Math.abs(error) > AlignmentZones.TOLERANCE) {
            integralSum *= 0.5;
        } else {
            integralSum += error * dt;
        }
        integralSum = Range.clip(integralSum, -AlignmentPID.INTEGRAL_LIMIT, AlignmentPID.INTEGRAL_LIMIT);
        double iTerm = AlignmentPID.KI * integralSum;

        // D term
        double derivative = (error - lastError) / dt;
        derivative = Range.clip(derivative, -50, 50);
        double dTerm = AlignmentPID.KD * derivative * gainMultiplier;
        lastError = error;

        double output = pTerm + iTerm + dTerm;
        output = Range.clip(output, -AlignmentSpeed.MAX_SPEED, AlignmentSpeed.MAX_SPEED);

        // CRITICAL: Only apply min speed when far from target AND min speed > 0
        if (Math.abs(error) > AlignmentZones.DEAD_ZONE &&
                Math.abs(output) > 0 &&
                Math.abs(output) < AlignmentSpeed.MIN_SPEED &&
                AlignmentSpeed.MIN_SPEED > 0) {  // Only if MIN_SPEED is not zero
            output = Math.signum(output) * AlignmentSpeed.MIN_SPEED;
        }

        return output;
    }

    /**
     * Apply acceleration limiting (unchanged)
     */
    private double applyAccelerationLimit(double targetPower) {
        double dt = accelerationTimer.seconds();
        accelerationTimer.reset();

        if (dt <= 0 || dt > 0.5) return targetPower;

        double maxChange = AlignmentSpeed.MAX_ACCELERATION * dt;
        double powerDiff = targetPower - lastOutput;

        if (Math.abs(powerDiff) > maxChange) {
            return lastOutput + Math.signum(powerDiff) * maxChange;
        }

        return targetPower;
    }

    /**
     * Smooth stop (unchanged)
     */
    private void smoothStop() {
        currentOutput *= 0.8;
        if (Math.abs(currentOutput) < 0.02) {
            currentOutput = 0;
            driveController.stopDrive();
        } else {
            applyTurn(currentOutput);
        }
        lastOutput = currentOutput;
    }

    /**
     * Apply turn power (unchanged)
     */
    private void applyTurn(double turnPower) {
        if (Math.abs(turnPower) > 0 && Math.abs(turnPower) < 0.05) {
            turnPower = 0;
        }
        driveController.tankDrive(turnPower, -turnPower);
    }

    /**
     * Maintain alignment when aligned (unchanged)
     */
    private void maintainAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
            wasAligning = false;  // Reset alignment tracking
            return;
        }

        double absError = Math.abs(filteredTx);

        if (absError > AlignmentZones.DEAD_ZONE) {
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            wasAligning = true;  // We're now aligning
            return;
        }

        if (absError > AlignmentZones.TOLERANCE) {
            double tinyCorrection = AlignmentPID.KP * filteredTx * 0.3;
            tinyCorrection = Range.clip(tinyCorrection, -0.05, 0.05);
            currentOutput = (0.9 * tinyCorrection) + (0.1 * lastOutput);
            lastOutput = currentOutput;
            applyTurn(currentOutput);
        } else {
            smoothStop();
        }
    }

    /**
     * ENHANCED: Handle lost target with search logic
     */
    private void handleTargetLost() {
        // If search is enabled and we were previously aligning, start search
        if (TargetSearch.ENABLE_SEARCH && wasAligning && stateTimer.milliseconds() > TargetSearch.SEARCH_DELAY_MS) {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
            return;
        }

        // Default behavior: smooth stop and wait
        handleTargetLostNoSearch();
    }

    /**
     * Handle target lost without search (original behavior)
     */
    private void handleTargetLostNoSearch() {
        smoothStop();
        consecutiveAlignedFrames = 0;

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            integralSum *= 0.5;
            wasAligning = true;
        }
    }

    /**
     * ENHANCED: Update state transitions with search logic
     */
    private void updateState() {
        // Handle target found during search
        if (currentState == AlignmentState.SEARCHING && hasTarget) {
            currentState = AlignmentState.ALIGNING;
            resetSearchState();
            return;
        }

        // Handle target lost
        if (!hasTarget && currentState != AlignmentState.TARGET_LOST &&
                currentState != AlignmentState.SEARCHING && currentState != AlignmentState.STOPPED) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
            // Don't reset wasAligning here - we need it for search decision
        }
        // Handle target found from lost state
        else if (hasTarget && currentState == AlignmentState.TARGET_LOST) {
            currentState = AlignmentState.ALIGNING;
            integralSum *= 0.5;
            lastError *= 0.5;
            wasAligning = true;
            resetSearchState();
        }
        // Track when we start aligning
        else if (currentState == AlignmentState.ALIGNING) {
            wasAligning = true;
        }
    }

    /**
     * Update tracking from Limelight (unchanged)
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
                boolean previousHasTarget = hasTarget;
                hasTarget = false;

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetTagId) {
                        hasTarget = true;
                        tx = fiducial.getTargetXDegrees();

                        if (!previousHasTarget) {
                            filteredTx = tx;
                        } else {
                            filteredTx = (AlignmentFiltering.INPUT_FILTER * tx) +
                                    ((1 - AlignmentFiltering.INPUT_FILTER) * filteredTx);
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

    /**
     * Update performance metrics (unchanged)
     */
    private void updateMetrics() {
        if (hasTarget) {
            double absError = Math.abs(filteredTx);
            maxError = Math.max(maxError, absError);
            minError = Math.min(minError, absError);
            avgError = (avgError * errorSamples + absError) / (errorSamples + 1);
            errorSamples++;
        }
    }

    /**
     * Reset PID state (unchanged)
     */
    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        lastOutput = 0;
        currentOutput = 0;
        pidTimer.reset();
        accelerationTimer.reset();
    }

    /**
     * NEW: Reset search state
     */
    private void resetSearchState() {
        searchTimer.reset();
    }

    /**
     * Reset performance metrics (unchanged)
     */
    public void resetMetrics() {
        maxError = 0;
        minError = Double.MAX_VALUE;
        avgError = 0;
        errorSamples = 0;
    }

    /**
     * ENHANCED: Send telemetry to dashboard with search info
     */
    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Alignment_State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("TX_Raw", tx);
        packet.put("TX_Filtered", filteredTx);
        packet.put("Error_Abs", Math.abs(filteredTx));
        packet.put("Output", currentOutput);
        packet.put("P_Term", AlignmentPID.KP * filteredTx);
        packet.put("I_Term", AlignmentPID.KI * integralSum);
        packet.put("Integral_Sum", integralSum);
        packet.put("Aligned_Frames", consecutiveAlignedFrames);

        // NEW: Search telemetry
        packet.put("Was_Aligning", wasAligning);
        packet.put("Search_Time_MS", searchTimer.milliseconds());
        packet.put("Search_Enabled", TargetSearch.ENABLE_SEARCH);

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * ENHANCED: Display telemetry to driver station with search info
     */
    public void displayTelemetry() {
        opMode.telemetry.addLine("═══ ALIGNMENT ═══");
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target", hasTarget ? "FOUND" : "LOST");

        if (hasTarget) {
            opMode.telemetry.addData("Error", "%.2f°", Math.abs(filteredTx));
            opMode.telemetry.addData("Output", "%.3f", currentOutput);
        }

        // NEW: Search status
        if (currentState == AlignmentState.SEARCHING) {
            opMode.telemetry.addData("Search Time", "%.1fs", searchTimer.seconds());
            opMode.telemetry.addData("Search Max", "%.1fs", TargetSearch.MAX_SEARCH_TIME_MS / 1000.0);
        }

        // Show PID values for verification
        opMode.telemetry.addLine();
        opMode.telemetry.addLine("── PID VALUES ──");
        opMode.telemetry.addData("KP", "%.4f", AlignmentPID.KP);
        opMode.telemetry.addData("KI", "%.4f", AlignmentPID.KI);
        opMode.telemetry.addData("KD", "%.4f", AlignmentPID.KD);
        opMode.telemetry.addData("MIN_SPEED", "%.2f", AlignmentSpeed.MIN_SPEED);

        // NEW: Search settings
        opMode.telemetry.addLine();
        opMode.telemetry.addLine("── SEARCH ──");
        opMode.telemetry.addData("Search Enabled", TargetSearch.ENABLE_SEARCH);
        opMode.telemetry.addData("Search Speed", "%.2f", TargetSearch.SEARCH_SPEED);
    }
}