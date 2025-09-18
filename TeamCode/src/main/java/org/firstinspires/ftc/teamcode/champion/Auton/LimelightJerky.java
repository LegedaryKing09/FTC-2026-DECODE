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
 * Continuously corrects the robot's heading to keep an AprilTag centered.
 * - Assumes the target is in view at the start.
 * - Does NOT control distance; it only rotates.
 * - The goal is to drive the horizontal offset (tx) to zero.
 */
@Config
@Autonomous(name = "AprilTag Centering (Heading Only)", group = "Champion")
public class LimelightJerky extends LinearOpMode {

    // Hardware
    private SixWheelDriveController driveController;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    // Target
    private static final int TARGET_TAG_ID = 20;

    @Config
    public static class TrackingParams {
        // PID gains for heading control (centering the tag)
        public static double KP_HEADING = 0.025;
        public static double KI_HEADING = 0.0;
        public static double KD_HEADING = 0.015;

        // Dead zone to prevent oscillation when centered
        public static double HEADING_DEAD_ZONE = 1.0;     // Stop turning if within 1.0 degree

        // Speed limits
        public static double MAX_TURN_SPEED = 0.6;
        public static double MIN_TURN_SPEED = 0.08;
    }

    // Tracking state
    private double tx = 0; // Horizontal offset from crosshair to target
    private boolean hasTarget = false;

    // PID state for heading
    private ElapsedTime headingPidTimer = new ElapsedTime();
    private double headingIntegralSum = 0;
    private double lastHeadingError = 0;

    // Robot state
    private enum CenteringState {
        CENTERING,      // Actively turning to center the tag
        CENTERED,       // Within the dead zone, holding position
        TARGET_LOST     // Target is not visible
    }
    private CenteringState currentState = CenteringState.CENTERING;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("═══ APRILTAG CENTERING ═══");
        telemetry.addLine(" (Heading Correction Only)");
        telemetry.addLine();
        telemetry.addLine("• Assumes target is in view.");
        telemetry.addLine("• Will only rotate to center tag.");
        telemetry.addLine("• Target ID: " + TARGET_TAG_ID);
        telemetry.addLine();
        telemetry.addLine(">>> Press START <<<");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        headingPidTimer.reset();

        while (opModeIsActive()) {
            updateTracking(); // Get the latest `tx` value from Limelight

            // State machine to control robot behavior
            switch (currentState) {
                case CENTERING:
                    executeCentering();
                    break;
                case CENTERED:
                    maintainCenter();
                    break;
                case TARGET_LOST:
                    // If target is lost, stop all movement for safety
                    driveController.stopDrive();
                    if (hasTarget) { // Recovered!
                        currentState = CenteringState.CENTERING;
                    }
                    break;
            }

            displayTelemetry();
            sendDashboardData();
            sleep(10); // Loop at ~100Hz
        }

        driveController.stopDrive();
    }

    /**
     * Main control loop for actively turning to center the AprilTag.
     */
    private void executeCentering() {
        if (!hasTarget) {
            currentState = CenteringState.TARGET_LOST;
            return;
        }

        double headingError = tx; // The error is the horizontal offset

        // Check if we are within the dead zone
        if (Math.abs(headingError) < TrackingParams.HEADING_DEAD_ZONE) {
            driveController.stopDrive();
            currentState = CenteringState.CENTERED;
            return;
        }

        // Calculate the power needed to correct the heading
        double turnPower = calculateHeadingPID(headingError);

        // Apply power to motors for turning
        driveController.tankDrive(turnPower, -turnPower);
    }

    /**
     * Calculates the required turning power using a PID controller.
     */
    private double calculateHeadingPID(double headingError) {
        double dt = headingPidTimer.seconds();
        headingPidTimer.reset();
        if (dt <= 0) return 0;

        // P (Proportional) term
        double pTerm = TrackingParams.KP_HEADING * headingError;

        // I (Integral) term - helps overcome resistance, accumulates error over time
        headingIntegralSum += headingError * dt;
        headingIntegralSum = Range.clip(headingIntegralSum, -10, 10); // Prevent integral windup
        double iTerm = TrackingParams.KI_HEADING * headingIntegralSum;

        // D (Derivative) term - dampens oscillation by reacting to the rate of change
        double dTerm = TrackingParams.KD_HEADING * (headingError - lastHeadingError) / dt;
        lastHeadingError = headingError;

        double output = pTerm + iTerm + dTerm;

        // Apply speed limits
        output = Range.clip(output, -TrackingParams.MAX_TURN_SPEED, TrackingParams.MAX_TURN_SPEED);

        // Apply a minimum power to overcome static friction
        if (Math.abs(output) > 0 && Math.abs(output) < TrackingParams.MIN_TURN_SPEED) {
            output = Math.signum(output) * TrackingParams.MIN_TURN_SPEED;
        }

        return output;
    }

    /**
     * Called when the robot is centered. Stops the robot and monitors
     * if it has drifted out of the dead zone.
     */
    private void maintainCenter() {
        if (!hasTarget) {
            currentState = CenteringState.TARGET_LOST;
            return;
        }

        // If we drift outside the allowed dead zone, go back to centering
        if (Math.abs(tx) > TrackingParams.HEADING_DEAD_ZONE) {
            currentState = CenteringState.CENTERING;
            lastHeadingError = tx; // Seed the derivative term
            return;
        }

        // Otherwise, stay stopped
        driveController.stopDrive();
    }

    /**
     * Fetches the latest data from the Limelight.
     * This version only cares about `tx` and whether a target is visible.
     */
    private void updateTracking() {
        if (limelight == null) {
            hasTarget = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            hasTarget = false; // Assume no target until we find the correct one

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                    hasTarget = true;
                    tx = fiducial.getTargetXDegrees();
                    break; // Found our target, no need to look further
                }
            }
        } else {
            hasTarget = false;
        }
    }

    // --- Initialization and Telemetry ---

    private void initializeHardware() {
        try {
            driveController = new SixWheelDriveController(this);
        } catch (Exception e) {
            telemetry.addLine("ERROR Initializing Drivetrain: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
        }

        dashboard = FtcDashboard.getInstance();

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Ensure AprilTag pipeline is active
            limelight.start();
        } catch (Exception e) {
            telemetry.addLine("WARNING: Limelight not found.");
            telemetry.update();
        }
    }

    private void displayTelemetry() {
        telemetry.clear();
        telemetry.addLine("═══ APRILTAG CENTERING ═══");
        telemetry.addData("State", currentState);
        telemetry.addData("Target Found", hasTarget ? "✅" : "❌");
        if (hasTarget) {
            telemetry.addData("Heading Error (tx)", "%.2f°", tx);
        }

        if (currentState == CenteringState.CENTERED) {
            telemetry.addLine();
            telemetry.addLine("🎯 CENTERED 🎯");
        }
        telemetry.update();
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("State", currentState.toString());
        packet.put("Has Target", hasTarget);
        packet.put("Heading Error (tx)", tx);
        dashboard.sendTelemetryPacket(packet);
    }
}