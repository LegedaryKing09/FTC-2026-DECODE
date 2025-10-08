package org.firstinspires.ftc.teamcode.champion.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

import java.util.List;

/**
 * Smooth AprilTag heading alignment with anti-jerk measures
 * Implements input filtering, output smoothing, and acceleration limiting
 */
@Config
@Autonomous(name = "Working Aligner", group = "Champion")
public class LimelightWORKING extends LinearOpMode {

    private SixWheelDriveController driveController;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    private static final int TARGET_TAG_ID = 20;

    @Config
    public static class HeadingParams {
        // PID gains - reduced for smoother control
        public static double KP_HEADING = 0.04;  // Reduced from 0.025
        public static double KI_HEADING = 0.0005;  // Reduced from 0.001
        public static double KD_HEADING = 0.008;   // Reduced from 0.015

        // Dead zone and tolerances
        public static double HEADING_TOLERANCE = 0.5;     // Perfect alignment
        public static double HEADING_DEAD_ZONE = 2.0;     // Increased slightly

        // Speed limits
        public static double MAX_TURN_SPEED = 0.5;        // Reduced from 0.8
        public static double MIN_TURN_SPEED = 0.3;       // Much lower minimum

        // Smoothing parameters
        public static double INPUT_FILTER_ALPHA = 0.7;    // Input exponential filter (0-1)
        public static double OUTPUT_FILTER_ALPHA = 0.8;   // Output exponential filter
        public static double MAX_ACCELERATION = 2.0;      // Max change per second

        // Integral windup limit
        public static double INTEGRAL_LIMIT = 3.0;        // Reduced from 5.0

        // Adaptive gain based on error magnitude
        public static boolean USE_ADAPTIVE_GAIN = true;
        public static double ADAPTIVE_GAIN_FACTOR = 0.8;  // Reduces gain when close
    }

    // Tracking state with filtering
    private double tx = 0;
    private double ty = 0;
    private double filteredTx = 0;  // Filtered heading error
    private boolean hasTarget = false;

    // PID state for heading
    private ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // Smoothing state
    private double lastOutput = 0;
    private double currentOutput = 0;
    private ElapsedTime accelerationTimer = new ElapsedTime();

    // State tracking
    private enum AlignmentState {
        ALIGNING,
        ALIGNED,
        TARGET_LOST
    }
    private AlignmentState currentState = AlignmentState.ALIGNING;

    private ElapsedTime stateTimer = new ElapsedTime();
    private int consecutiveAlignedFrames = 0;
    private static final int ALIGNED_FRAME_THRESHOLD = 15; // Increased for stability

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("═══ SMOOTH ALIGNER ═══");
        telemetry.addLine();
        telemetry.addLine("• Anti-jerk AprilTag alignment");
        telemetry.addLine("• Target: Tag " + TARGET_TAG_ID);
        telemetry.addLine();
        telemetry.addLine(">>> Press START <<<");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        pidTimer.reset();
        accelerationTimer.reset();

        while (opModeIsActive()) {

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
            }

            displayTelemetry();
            sendDashboardData();

            sleep(20); // 50Hz update rate - smoother than 100Hz
        }

        driveController.stopDrive();
    }

    /**
     * Smoothed heading alignment with anti-jerk measures
     */
    private void executeSmoothedAlignment() {
        if (!hasTarget) {
            // Smoothly ramp down to stop
            smoothStop();
            return;
        }

        double headingError = filteredTx; // Use filtered value
        double absError = Math.abs(headingError);

        // Check if we're aligned
        if (absError <= HeadingParams.HEADING_TOLERANCE) {
            consecutiveAlignedFrames++;
            if (consecutiveAlignedFrames >= ALIGNED_FRAME_THRESHOLD) {
                currentState = AlignmentState.ALIGNED;
                stateTimer.reset();
                smoothStop();
                return;
            }
        } else {
            consecutiveAlignedFrames = Math.max(0, consecutiveAlignedFrames - 2); // Gradual reset
        }

        // Calculate base PID output
        double turnPower = calculateSmoothedPID(headingError);

        // Apply progressive scaling near the target (smooth approach)
        if (absError < HeadingParams.HEADING_DEAD_ZONE) {
            double scale = absError / HeadingParams.HEADING_DEAD_ZONE;
            scale = Math.pow(scale, 1.5); // Non-linear scaling for smoother approach
            turnPower *= scale;
        }

        // Apply acceleration limiting
        turnPower = applyAccelerationLimit(turnPower);

        // Apply output filtering
        currentOutput = (HeadingParams.OUTPUT_FILTER_ALPHA * turnPower) +
                ((1 - HeadingParams.OUTPUT_FILTER_ALPHA) * lastOutput);
        lastOutput = currentOutput;

        // Apply the smoothed power
        applyTurn(currentOutput);
    }

    /**
     * Calculate PID with adaptive gain and smoothing
     */
    private double calculateSmoothedPID(double error) {
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;

        // Handle first iteration
        if (dt <= 0 || lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return HeadingParams.KP_HEADING * error;
        }

        lastTime = currentTime;

        // Adaptive gain reduction when close to target
        double gainMultiplier = 1.0;
        if (HeadingParams.USE_ADAPTIVE_GAIN) {
            double absError = Math.abs(error);
            if (absError < HeadingParams.HEADING_DEAD_ZONE * 2) {
                gainMultiplier = HeadingParams.ADAPTIVE_GAIN_FACTOR +
                        (1 - HeadingParams.ADAPTIVE_GAIN_FACTOR) *
                                (absError / (HeadingParams.HEADING_DEAD_ZONE * 2));
            }
        }

        // Proportional term with adaptive gain
        double pTerm = HeadingParams.KP_HEADING * error * gainMultiplier;

        // Integral term with conditional integration (reduce when oscillating)
        if (Math.signum(error) != Math.signum(lastError) && Math.abs(error) > HeadingParams.HEADING_TOLERANCE) {
            integralSum *= 0.5; // Reduce integral when crossing zero (oscillating)
        } else {
            integralSum += error * dt;
        }
        integralSum = Range.clip(integralSum,
                -HeadingParams.INTEGRAL_LIMIT,
                HeadingParams.INTEGRAL_LIMIT);
        double iTerm = HeadingParams.KI_HEADING * integralSum;

        // Derivative term with filtering
        double derivative = (error - lastError) / dt;
        // Simple derivative filter to reduce noise
        derivative = Range.clip(derivative, -50, 50); // Limit extreme derivatives
        double dTerm = HeadingParams.KD_HEADING * derivative * gainMultiplier;
        lastError = error;

        // Calculate total output
        double output = pTerm + iTerm + dTerm;

        // Apply speed limits
        output = Range.clip(output, -HeadingParams.MAX_TURN_SPEED, HeadingParams.MAX_TURN_SPEED);

        // Apply minimum speed only when far from target
        if (Math.abs(error) > HeadingParams.HEADING_DEAD_ZONE &&
                Math.abs(output) > 0 && Math.abs(output) < HeadingParams.MIN_TURN_SPEED) {
            output = Math.signum(output) * HeadingParams.MIN_TURN_SPEED;
        }

        return output;
    }

    /**
     * Apply acceleration limiting for smooth speed changes
     */
    private double applyAccelerationLimit(double targetPower) {
        double dt = accelerationTimer.seconds();
        accelerationTimer.reset();

        if (dt <= 0 || dt > 0.5) { // Sanity check
            return targetPower;
        }

        double maxChange = HeadingParams.MAX_ACCELERATION * dt;
        double powerDiff = targetPower - lastOutput;

        if (Math.abs(powerDiff) > maxChange) {
            return lastOutput + Math.signum(powerDiff) * maxChange;
        }

        return targetPower;
    }

    /**
     * Smoothly stop the robot
     */
    private void smoothStop() {
        // Gradually reduce power to zero
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
     * Apply turn power to motors
     */
    private void applyTurn(double turnPower) {
        // Add small deadband compensation for motor controllers
        if (Math.abs(turnPower) > 0 && Math.abs(turnPower) < 0.05) {
            turnPower = 0;
        }
        driveController.tankDrive(turnPower, -turnPower);
    }

    /**
     * Maintain alignment with minimal corrections
     */
    private void maintainAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
            return;
        }

        double absError = Math.abs(filteredTx);

        // Check if we've drifted significantly
        if (absError > HeadingParams.HEADING_DEAD_ZONE) {
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            // Don't reset PID to maintain continuity
            return;
        }

        // Make tiny smooth corrections
        if (absError > HeadingParams.HEADING_TOLERANCE) {
            double tinyCorrection = HeadingParams.KP_HEADING * filteredTx * 0.3;
            tinyCorrection = Range.clip(tinyCorrection, -0.05, 0.05);

            // Apply output filtering even for tiny corrections
            currentOutput = (0.9 * tinyCorrection) + (0.1 * lastOutput);
            lastOutput = currentOutput;
            applyTurn(currentOutput);
        } else {
            smoothStop();
        }
    }

    /**
     * Handle lost target with smooth stop
     */
    private void handleTargetLost() {
        smoothStop();
        consecutiveAlignedFrames = 0;

        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            // Soft reset - keep some state for continuity
            integralSum *= 0.5;
        }
    }

    /**
     * Update state based on conditions
     */
    private void updateState() {
        if (!hasTarget && currentState != AlignmentState.TARGET_LOST) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
        } else if (hasTarget && currentState == AlignmentState.TARGET_LOST) {
            currentState = AlignmentState.ALIGNING;
            // Soft reset for smooth transition
            integralSum *= 0.5;
            lastError *= 0.5;
        }
    }

    /**
     * Update tracking data with filtering
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
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        hasTarget = true;
                        tx = fiducial.getTargetXDegrees();
                        ty = fiducial.getTargetYDegrees();

                        // Apply input filtering
                        if (!previousHasTarget) {
                            // First detection - initialize filter
                            filteredTx = tx;
                        } else {
                            // Exponential moving average filter
                            filteredTx = (HeadingParams.INPUT_FILTER_ALPHA * tx) +
                                    ((1 - HeadingParams.INPUT_FILTER_ALPHA) * filteredTx);
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

    private void initializeHardware() {
        try {
            driveController = new SixWheelDriveController(this);
        } catch (Exception e) {
            telemetry.addLine("ERROR: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            throw e;
        }

        dashboard = FtcDashboard.getInstance();

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            telemetry.addLine("WARNING: Limelight not found");
            telemetry.update();
        }
    }

    private void displayTelemetry() {
        telemetry.clear();
        telemetry.addLine("═══ SMOOTH ALIGNMENT ═══");
        telemetry.addLine();

        telemetry.addData("State", currentState);
        telemetry.addData("Target Found", hasTarget ? "YES" : "NO");

        if (hasTarget) {
            telemetry.addData("Raw TX", "%.2f°", tx);
            telemetry.addData("Filtered TX", "%.2f°", filteredTx);
            telemetry.addData("Output Power", "%.3f", currentOutput);

            double absError = Math.abs(filteredTx);
            if (absError <= HeadingParams.HEADING_TOLERANCE) {
                telemetry.addLine("✓ PERFECTLY ALIGNED ✓");
            } else if (absError <= HeadingParams.HEADING_DEAD_ZONE) {
                telemetry.addLine("→ FINE TUNING ←");
            } else {
                telemetry.addLine(">> ALIGNING <<");
            }

            telemetry.addData("Integral Sum", "%.3f", integralSum);
            telemetry.addData("Aligned Frames", "%d/%d",
                    consecutiveAlignedFrames, ALIGNED_FRAME_THRESHOLD);
        }

        if (currentState == AlignmentState.ALIGNED) {
            telemetry.addLine("═══ TARGET CENTERED ═══");
            telemetry.addData("Time Aligned", "%.1fs", stateTimer.seconds());
        }

        telemetry.update();
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("State", currentState.toString());
        packet.put("Has_Target", hasTarget);
        packet.put("Raw_TX", tx);
        packet.put("Filtered_TX", filteredTx);
        packet.put("TX_Error_Abs", Math.abs(filteredTx));
        packet.put("Output_Power", currentOutput);
        packet.put("Integral_Sum", integralSum);
        packet.put("Aligned_Frames", consecutiveAlignedFrames);

        if (currentState == AlignmentState.ALIGNED) {
            packet.put("Time_Aligned", stateTimer.seconds());
        }

        dashboard.sendTelemetryPacket(packet);
    }
}