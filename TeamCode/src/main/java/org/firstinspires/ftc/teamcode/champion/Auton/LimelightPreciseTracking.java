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
 * Continuous AprilTag heading alignment - keeps tag centered (tx = 0)
 * Assumes target is in view at start
 * No distance control, only heading alignment
 */
@Config
@Autonomous(name = "AprilTag Heading Aligner", group = "Champion")
public class LimelightPreciseTracking extends LinearOpMode {

    private SixWheelDriveController driveController;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    private static final int TARGET_TAG_ID = 20;

    @Config
    public static class HeadingParams {
        // PID gains for heading control (tx = 0)
        public static double KP_HEADING = 0.025;
        public static double KI_HEADING = 0.001;
        public static double KD_HEADING = 0.015;

        // Dead zone and tolerances
        public static double HEADING_DEAD_ZONE = 1.5;     // Stop turning if within 1.0 degrees
        public static double HEADING_TOLERANCE = 0.5;     // Perfect alignment threshold

        // Speed limits
        public static double MAX_TURN_SPEED = 0.8;
        public static double MIN_TURN_SPEED = 0.4;

        // Integral windup limit
        public static double INTEGRAL_LIMIT = 5.0;
    }

    // Tracking state
    private double tx = 0;
    private double ty = 0;
    private boolean hasTarget = false;

    // PID state for heading
    private ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // State tracking
    private enum AlignmentState {
        ALIGNING,
        ALIGNED,
        TARGET_LOST
    }
    private AlignmentState currentState = AlignmentState.ALIGNING;

    private ElapsedTime stateTimer = new ElapsedTime();
    private int consecutiveAlignedFrames = 0;
    private static final int ALIGNED_FRAME_THRESHOLD = 10; // Must be aligned for 10 frames

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("═══ HEADING ALIGNER ═══");
        telemetry.addLine();
        telemetry.addLine("• Assumes target in view");
        telemetry.addLine("• Keeps AprilTag centered (tx=0)");
        telemetry.addLine("• Target: Tag " + TARGET_TAG_ID);
        telemetry.addLine();
        telemetry.addLine(">>> Press START <<<");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        pidTimer.reset();

        while (opModeIsActive()) {

            updateTracking();
            updateState();

            switch (currentState) {
                case ALIGNING:
                    executeHeadingAlignment();
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

            sleep(10); // 100Hz update rate
        }

        driveController.stopDrive();
    }

    /**
     * Main heading alignment - keeps AprilTag centered
     */
    private void executeHeadingAlignment() {
        if (!hasTarget) {
            driveController.stopDrive();
            return;
        }

        double headingError = tx; // Want tx = 0
        double absError = Math.abs(headingError);

        // Check if we're within dead zone
        if (absError <= HeadingParams.HEADING_DEAD_ZONE) {
            // Within dead zone - check for perfect alignment
            if (absError <= HeadingParams.HEADING_TOLERANCE) {
                consecutiveAlignedFrames++;
                if (consecutiveAlignedFrames >= ALIGNED_FRAME_THRESHOLD) {
                    currentState = AlignmentState.ALIGNED;
                    stateTimer.reset();
                    driveController.stopDrive();
                    return;
                }
            } else {
                consecutiveAlignedFrames = 0;
            }

            // Apply small correction if not perfectly aligned
            double smallCorrection = HeadingParams.KP_HEADING * headingError * 0.5;
            smallCorrection = Range.clip(smallCorrection, -0.1, 0.1);
            applyTurn(smallCorrection);
        } else {
            // Outside dead zone - full PID control
            consecutiveAlignedFrames = 0;
            double turnPower = calculateHeadingPID(headingError);
            applyTurn(turnPower);
        }
    }

    /**
     * Calculate PID output for heading control
     */
    private double calculateHeadingPID(double error) {
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;

        // Prevent divide by zero and handle first iteration
        if (dt <= 0 || lastTime == 0) {
            lastTime = currentTime;
            lastError = error;
            return HeadingParams.KP_HEADING * error;
        }

        lastTime = currentTime;

        // Proportional term
        double pTerm = HeadingParams.KP_HEADING * error;

        // Integral term with anti-windup
        integralSum += error * dt;
        integralSum = Range.clip(integralSum,
                -HeadingParams.INTEGRAL_LIMIT,
                HeadingParams.INTEGRAL_LIMIT);
        double iTerm = HeadingParams.KI_HEADING * integralSum;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double dTerm = HeadingParams.KD_HEADING * derivative;
        lastError = error;

        // Calculate total output
        double output = pTerm + iTerm + dTerm;

        // Apply speed limits
        output = Range.clip(output, -HeadingParams.MAX_TURN_SPEED, HeadingParams.MAX_TURN_SPEED);

        // Apply minimum speed threshold (deadband compensation)
        if (Math.abs(output) > 0 && Math.abs(output) < HeadingParams.MIN_TURN_SPEED) {
            output = Math.signum(output) * HeadingParams.MIN_TURN_SPEED;
        }

        return output;
    }

    /**
     * Apply turn power to motors (tank drive)
     */
    private void applyTurn(double turnPower) {
        // Positive turnPower = turn right (to center a target on the left)
        // Negative turnPower = turn left (to center a target on the right)
        driveController.tankDrive(turnPower, -turnPower);
    }

    /**
     * Maintain alignment when already aligned
     */
    private void maintainAlignment() {
        if (!hasTarget) {
            currentState = AlignmentState.TARGET_LOST;
            stateTimer.reset();
            consecutiveAlignedFrames = 0;
            return;
        }

        double absError = Math.abs(tx);

        // Check if we've drifted out of alignment
        if (absError > HeadingParams.HEADING_DEAD_ZONE * 1.5) {
            // Significant drift - go back to aligning
            currentState = AlignmentState.ALIGNING;
            consecutiveAlignedFrames = 0;
            resetPIDState();
            return;
        }

        // Make tiny corrections to maintain perfect alignment
        if (absError > HeadingParams.HEADING_TOLERANCE) {
            double tinyCorrection = HeadingParams.KP_HEADING * tx * 0.3;
            tinyCorrection = Range.clip(tinyCorrection, -0.08, 0.08);
            applyTurn(tinyCorrection);
        } else {
            driveController.stopDrive();
        }
    }

    /**
     * Handle lost target
     */
    private void handleTargetLost() {
        driveController.stopDrive();
        consecutiveAlignedFrames = 0;

        // If target comes back, resume alignment
        if (hasTarget) {
            currentState = AlignmentState.ALIGNING;
            resetPIDState();
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
            resetPIDState();
        }
    }

    /**
     * Reset PID state
     */
    private void resetPIDState() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        pidTimer.reset();
    }

    /**
     * Update tracking data from Limelight
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
                hasTarget = false;

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        hasTarget = true;
                        tx = fiducial.getTargetXDegrees();
                        ty = fiducial.getTargetYDegrees();
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
        telemetry.addLine("═══ HEADING ALIGNMENT ═══");
        telemetry.addLine();

        telemetry.addData("State", currentState);
        telemetry.addData("Target Found", hasTarget ? "YES" : "NO");

        if (hasTarget) {
            telemetry.addData("TX (heading error)", "%.2f°", tx);
            telemetry.addData("TY", "%.2f°", ty);

            double absError = Math.abs(tx);
            if (absError <= HeadingParams.HEADING_TOLERANCE) {
                telemetry.addLine("✓ PERFECTLY ALIGNED ✓");
            } else if (absError <= HeadingParams.HEADING_DEAD_ZONE) {
                telemetry.addLine("→ FINE TUNING ←");
            } else {
                telemetry.addLine(">> ALIGNING <<");
            }

            // Show PID components for debugging
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
        packet.put("TX", tx);
        packet.put("TY", ty);
        packet.put("TX_Error_Abs", Math.abs(tx));
        packet.put("Integral_Sum", integralSum);
        packet.put("Aligned_Frames", consecutiveAlignedFrames);

        if (currentState == AlignmentState.ALIGNED) {
            packet.put("Time_Aligned", stateTimer.seconds());
        }

        dashboard.sendTelemetryPacket(packet);
    }
}