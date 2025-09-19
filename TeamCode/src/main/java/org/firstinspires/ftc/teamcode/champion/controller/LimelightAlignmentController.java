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
 * Smooth AprilTag alignment controller with Dashboard-tunable PID
 * Can be used in both autonomous and teleop modes
 */
@Config
public class LimelightAlignmentController {

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final Limelight3A limelight;
    private final FtcDashboard dashboard;

    @Config
    public static class PID {
        // PID gains - ALL DASHBOARD ADJUSTABLE
        public static double KP = 0.04;
        public static double KI = 0.0005;
        public static double KD = 0.008;

        // Integral windup limit
        public static double INTEGRAL_LIMIT = 3.0;
    }

    @Config
    public static class Zones {
        // Dead zones and tolerances
        public static double TOLERANCE = 0.5;      // Perfect alignment threshold
        public static double DEAD_ZONE = 2.0;      // Stop turning threshold
    }

    @Config
    public static class Speed {
        // Speed limits
        public static double MAX_SPEED = 0.5;
        public static double MIN_SPEED = 0.3;
        public static double MAX_ACCELERATION = 2.0;  // Max change per second
    }

    @Config
    public static class Filtering {
        // Smoothing parameters
        public static double INPUT_FILTER = 0.7;   // Input filter (0-1, higher=less filtering)
        public static double OUTPUT_FILTER = 0.8;  // Output filter
    }

    @Config
    public static class Advanced {
        // Advanced tuning
        public static boolean ADAPTIVE_GAIN = true;
        public static double ADAPTIVE_FACTOR = 0.8;
        public static int ALIGNED_FRAMES = 15;     // Frames to confirm alignment
        public static double PROGRESSIVE_POWER = 1.5; // Power curve exponent
    }

    // Tracking state with filtering
    private double tx = 0;
    private double ty = 0;
    private double filteredTx = 0;
    private boolean hasTarget = false;

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
        ALIGNING, ALIGNED, TARGET_LOST, STOPPED
    }
    private AlignmentState currentState = AlignmentState.STOPPED;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int consecutiveAlignedFrames = 0;

    // Performance metrics
    private double maxError = 0;
    private double minError = Double.MAX_VALUE;
    private double avgError = 0;
    private int errorSamples = 0;

    // Alignment result class
    public static class AlignmentResult {
        public final AlignmentState state;
        public final boolean hasTarget;
        public final double error;
        public final double output;
        public final boolean isAligned;

        public AlignmentResult(AlignmentState state, boolean hasTarget, double error, double output, boolean isAligned) {
            this.state = state;
            this.hasTarget = hasTarget;
            this.error = error;
            this.output = output;
            this.isAligned = isAligned;
        }
    }

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

        // Initialize timers
        pidTimer.reset();
        accelerationTimer.reset();
        stateTimer.reset();
    }

    /**
     * Main alignment method - call this continuously in your teleop/autonomous loop
     * @param targetTagId The AprilTag ID to align to
     * @return AlignmentResult containing current state and alignment info
     */
    public AlignmentResult align(int targetTagId) {
        updateTracking(targetTagId);
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
            case STOPPED:
                // Do nothing - manual control
                break;
        }

        updateMetrics();
        sendDashboardData();

        boolean isAligned = (currentState == AlignmentState.ALIGNED);
        return new AlignmentResult(currentState, hasTarget, Math.abs(filteredTx), currentOutput, isAligned);
    }

    /**
     * Start alignment process
     */
    public void startAlignment() {
        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
        } else {
            currentState = AlignmentState.TARGET_LOST;
        }
        consecutiveAlignedFrames = 0;
        resetPID();
    }

    /**
     * Stop alignment and return control to manual driving
     */
    public void stopAlignment() {
        currentState = AlignmentState.STOPPED;
        driveController.stopDrive();
        resetPID();
    }

    /**
     * Check if currently aligned to target
     */
    public boolean isAligned() {
        return currentState == AlignmentState.ALIGNED;
    }

    /**
     * Get current alignment state
     */
    public AlignmentState getState() {
        return currentState;
    }

    /**
     * Get current target info
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTargetError() {
        return Math.abs(filteredTx);
    }

    /**
     * Reset PID controller state
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
     * Reset performance metrics
     */
    public void resetMetrics() {
        maxError = 0;
        minError = Double.MAX_VALUE;
        avgError = 0;
        errorSamples = 0;
    }

    private void executeSmoothedAlignment() {
        if (!hasTarget) {
            smoothStop();
            return;
        }

        double headingError = filteredTx;
        double absError = Math.abs(headingError);

        // Check alignment
        if (absError <= Zones.TOLERANCE) {
            consecutiveAlignedFrames++;
            if (consecutiveAlignedFrames >= Advanced.ALIGNED_FRAMES) {
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
        if (absError < Zones.DEAD_ZONE) {
            double scale = absError / Zones.DEAD_ZONE;
            scale = Math.pow(scale, Advanced.PROGRESSIVE_POWER);
            turnPower *= scale;
        }

        // Apply acceleration limiting
        turnPower = applyAccelerationLimit(turnPower);

        // Apply output filtering
        currentOutput = (Filtering.OUTPUT_FILTER * turnPower) +
                ((1 - Filtering.OUTPUT_FILTER) * lastOutput);
        lastOutput = currentOutput;

        applyTurn(currentOutput);
    }

    private double calculateSmoothedPID(double error) {
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;

        if (dt <= 0 || lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return PID.KP * error;
        }

        lastTime = currentTime;

        // Adaptive gain
        double gainMultiplier = 1.0;
        if (Advanced.ADAPTIVE_GAIN) {
            double absError = Math.abs(error);
            if (absError < Zones.DEAD_ZONE * 2) {
                gainMultiplier = Advanced.ADAPTIVE_FACTOR +
                        (1 - Advanced.ADAPTIVE_FACTOR) * (absError / (Zones.DEAD_ZONE * 2));
            }
        }

        // P term
        double pTerm = PID.KP * error * gainMultiplier;

        // I term with anti-oscillation
        if (Math.signum(error) != Math.signum(lastError) && Math.abs(error) > Zones.TOLERANCE) {
            integralSum *= 0.5;
        } else {
            integralSum += error * dt;
        }
        integralSum = Range.clip(integralSum, -PID.INTEGRAL_LIMIT, PID.INTEGRAL_LIMIT);
        double iTerm = PID.KI * integralSum;

        // D term
        double derivative = (error - lastError) / dt;
        derivative = Range.clip(derivative, -50, 50);
        double dTerm = PID.KD * derivative * gainMultiplier;
        lastError = error;

        double output = pTerm + iTerm + dTerm;
        output = Range.clip(output, -Speed.MAX_SPEED, Speed.MAX_SPEED);

        // Min speed only when far from target
        if (Math.abs(error) > Zones.DEAD_ZONE &&
                Math.abs(output) > 0 && Math.abs(output) < Speed.MIN_SPEED) {
            output = Math.signum(output) * Speed.MIN_SPEED;
        }

        return output;
    }

    private double applyAccelerationLimit(double targetPower) {
        double dt = accelerationTimer.seconds();
        accelerationTimer.reset();

        if (dt <= 0 || dt > 0.5) return targetPower;

        double maxChange = Speed.MAX_ACCELERATION * dt;
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

        if (absError > Zones.DEAD_ZONE) {
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            return;
        }

        if (absError > Zones.TOLERANCE) {
            double tinyCorrection = PID.KP * filteredTx * 0.3;
            tinyCorrection = Range.clip(tinyCorrection, -0.05, 0.05);
            currentOutput = (0.9 * tinyCorrection) + (0.1 * lastOutput);
            lastOutput = currentOutput;
            applyTurn(currentOutput);
        } else {
            smoothStop();
        }
    }

    private void handleTargetLost() {
        smoothStop();
        consecutiveAlignedFrames = 0;

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            integralSum *= 0.5;
        }
    }

    private void updateState() {
        if (!hasTarget && currentState != AlignmentState.TARGET_LOST && currentState != AlignmentState.STOPPED) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
        } else if (hasTarget && currentState == AlignmentState.TARGET_LOST) {
            currentState = AlignmentState.ALIGNING;
            integralSum *= 0.5;
            lastError *= 0.5;
        }
    }

    private void updateTracking(int targetTagId) {
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
                        ty = fiducial.getTargetYDegrees();

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

    private void updateMetrics() {
        if (hasTarget) {
            double absError = Math.abs(filteredTx);
            maxError = Math.max(maxError, absError);
            minError = Math.min(minError, absError);
            avgError = (avgError * errorSamples + absError) / (errorSamples + 1);
            errorSamples++;
        }
    }

    private void sendDashboardData() {
        if (dashboard == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // Live data
        packet.put("Alignment_State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("TX_Raw", tx);
        packet.put("TX_Filtered", filteredTx);
        packet.put("Error_Abs", Math.abs(filteredTx));
        packet.put("Output", currentOutput);

        // PID components
        packet.put("P_Term", PID.KP * filteredTx);
        packet.put("I_Term", PID.KI * integralSum);
        packet.put("Integral_Sum", integralSum);

        // Metrics
        packet.put("Min_Error", minError);
        packet.put("Max_Error", maxError);
        packet.put("Avg_Error", avgError);
        packet.put("Aligned_Frames", consecutiveAlignedFrames);

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Display alignment telemetry to driver station
     */
    public void displayTelemetry() {
        opMode.telemetry.addLine("═══ ALIGNMENT ═══");
        opMode.telemetry.addData("State", currentState);
        opMode.telemetry.addData("Target", hasTarget ? "FOUND" : "LOST");

        if (hasTarget) {
            opMode.telemetry.addData("Error", "%.2f°", Math.abs(filteredTx));
            opMode.telemetry.addData("Output", "%.3f", currentOutput);
        }
    }
}