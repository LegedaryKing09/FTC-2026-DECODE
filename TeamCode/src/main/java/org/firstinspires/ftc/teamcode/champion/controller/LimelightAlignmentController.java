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
 * Smooth AprilTag alignment controller with counterclockwise target search
 * Modified to turn counterclockwise at 0.6 speed when searching
 */
@Config
public class LimelightAlignmentController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    @Config
    public static class AlignmentPID {
        public static double KP = 0.04;
        public static double KI = 0.0003;
        public static double KD = 0.02;
        public static double INTEGRAL_LIMIT = 3.0;
    }

    @Config
    public static class AlignmentZones {
        public static double TOLERANCE = 0.5;
        public static double DEAD_ZONE = 2.0;
    }

    @Config
    public static class AlignmentSpeed {
        public static double MAX_SPEED = 0.5;
        public static double MIN_SPEED = 0.0;
        public static double MAX_ACCELERATION = 1.5;
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

    @Config
    public static class TargetSearch {
        public static boolean ENABLE_SEARCH = true;
        public static double SEARCH_SPEED = 0.6;  // Increased from 0.25 to 0.6 as requested
        public static double SEARCH_DELAY_MS = 300;  // Reduced delay for faster response
        public static double MAX_SEARCH_TIME_MS = 5000;
    }

    // Tracking state
    private double tx = 0;
    private double filteredTx = 0;
    private boolean hasTarget = false;
    private int targetTagId = 20;

    // PID state
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // Smoothing state
    private double lastOutput = 0;
    private double currentOutput = 0;
    private final ElapsedTime accelerationTimer = new ElapsedTime();

    // State tracking
    public enum AlignmentState {
        ALIGNING, ALIGNED, TARGET_LOST, SEARCHING, STOPPED
    }
    private AlignmentState currentState = AlignmentState.STOPPED;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int consecutiveAlignedFrames = 0;

    // Search state tracking
    private final ElapsedTime searchTimer = new ElapsedTime();
    private boolean searchInitiated = false;  // Track if search was started

    // Performance metrics
    private double maxError = 0;
    private double minError = Double.MAX_VALUE;
    private double avgError = 0;
    private int errorSamples = 0;

    // Control flags
    private boolean isActive = false;

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

    public void setTargetTag(int tagId) {
        this.targetTagId = tagId;
    }

    public void startAlignment() {
        isActive = true;
        // Check if we have target immediately
        updateTracking();
        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
        } else {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
            searchInitiated = true;
        }
        consecutiveAlignedFrames = 0;
        resetPID();
    }

    public void stopAlignment() {
        isActive = false;
        currentState = AlignmentState.STOPPED;
        driveController.stopDrive();
        resetPID();
        resetSearchState();
    }

    public void align(int targetTagId) {
        if (!isActive) return;

        this.targetTagId = targetTagId;
        updateTracking();

        switch (currentState) {
            case SEARCHING:
                executeTargetSearch();
                break;
            case ALIGNING:
                executeSmoothedAlignment();
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

        updateMetrics();
        sendDashboardData();
    }

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
        return Math.abs(filteredTx);
    }

    /**
     * Execute counterclockwise target search at 0.6 speed
     */
    private void executeTargetSearch() {
        if (!TargetSearch.ENABLE_SEARCH) {
            currentState = AlignmentState.TARGET_LOST;
            smoothStop();
            return;
        }

        // Check if we found the target
        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            resetSearchState();
            resetPID();
            return;
        }

        // Check search timeout
        if (searchTimer.milliseconds() > TargetSearch.MAX_SEARCH_TIME_MS) {
            currentState = AlignmentState.TARGET_LOST;
            resetSearchState();
            smoothStop();
            return;
        }

        // Execute counterclockwise turn (negative power for left wheels, positive for right)
        double searchPower = TargetSearch.SEARCH_SPEED;
        driveController.tankDrive(-searchPower, searchPower);  // Counterclockwise turn

        // Update tracking for smooth output
        currentOutput = searchPower;
        lastOutput = currentOutput;
    }

    private void executeSmoothedAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
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

        // Only apply min speed when far from target AND min speed > 0
        if (Math.abs(error) > AlignmentZones.DEAD_ZONE &&
                Math.abs(output) > 0 &&
                Math.abs(output) < AlignmentSpeed.MIN_SPEED &&
                AlignmentSpeed.MIN_SPEED > 0) {
            output = Math.signum(output) * AlignmentSpeed.MIN_SPEED;
        }

        return output;
    }

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

    private void applyTurn(double turnPower) {
        if (Math.abs(turnPower) > 0 && Math.abs(turnPower) < 0.05) {
            turnPower = 0;
        }
        driveController.tankDrive(turnPower, -turnPower);
    }

    private void maintainAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
            return;
        }

        double absError = Math.abs(filteredTx);

        if (absError > AlignmentZones.DEAD_ZONE) {
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
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

    private void handleTargetLost() {
        // If search is enabled, start searching immediately
        if (TargetSearch.ENABLE_SEARCH && stateTimer.milliseconds() > TargetSearch.SEARCH_DELAY_MS) {
            currentState = AlignmentState.SEARCHING;
            searchTimer.reset();
            searchInitiated = true;
            return;
        }

        smoothStop();
        consecutiveAlignedFrames = 0;

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            integralSum *= 0.5;
        }
    }

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

    private void updateMetrics() {
        if (hasTarget) {
            double absError = Math.abs(filteredTx);
            maxError = Math.max(maxError, absError);
            minError = Math.min(minError, absError);
            avgError = (avgError * errorSamples + absError) / (errorSamples + 1);
            errorSamples++;
        }
    }

    public void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        lastOutput = 0;
        currentOutput = 0;
        pidTimer.reset();
        accelerationTimer.reset();
    }

    private void resetSearchState() {
        searchTimer.reset();
        searchInitiated = false;
    }

    public void resetMetrics() {
        maxError = 0;
        minError = Double.MAX_VALUE;
        avgError = 0;
        errorSamples = 0;
    }

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
        packet.put("Search_Time_MS", searchTimer.milliseconds());
        packet.put("Search_Enabled", TargetSearch.ENABLE_SEARCH);

        dashboard.sendTelemetryPacket(packet);
    }

    public void displayTelemetry() {
        opMode.telemetry.addLine("═══ ALIGNMENT ═══");
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target", hasTarget ? "FOUND" : "LOST");

        if (hasTarget) {
            opMode.telemetry.addData("Error", "%.2f°", Math.abs(filteredTx));
            opMode.telemetry.addData("Output", "%.3f", currentOutput);
        }

        if (currentState == AlignmentState.SEARCHING) {
            opMode.telemetry.addData("Search Time", "%.1fs", searchTimer.seconds());
            opMode.telemetry.addData("Search Max", "%.1fs", TargetSearch.MAX_SEARCH_TIME_MS / 1000.0);
        }

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("── PID VALUES ──");
        opMode.telemetry.addData("KP", "%.4f", AlignmentPID.KP);
        opMode.telemetry.addData("KI", "%.4f", AlignmentPID.KI);
        opMode.telemetry.addData("KD", "%.4f", AlignmentPID.KD);
        opMode.telemetry.addData("MIN_SPEED", "%.2f", AlignmentSpeed.MIN_SPEED);

        opMode.telemetry.addLine();
        opMode.telemetry.addLine("── SEARCH ──");
        opMode.telemetry.addData("Search Enabled", TargetSearch.ENABLE_SEARCH);
        opMode.telemetry.addData("Search Speed", "%.2f", TargetSearch.SEARCH_SPEED);
        opMode.telemetry.addData("Search Direction", "COUNTERCLOCKWISE");
    }
}