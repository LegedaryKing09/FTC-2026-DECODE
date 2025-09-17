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
 * Stable tracking without oscillation
 * - Dead zone to stop hunting
 * - Damped PID gains
 * - Velocity-based stopping
 */
@Config
@Autonomous(name = "Working Well Angle", group = "Champion")
public class LimelightPreciseTracking extends LinearOpMode {

    private SixWheelDriveController driveController;
    private Limelight3A limelight;
    private FtcDashboard dashboard;

    private static final int TARGET_TAG_ID = 20;
    private static final double TARGET_DISTANCE = 48.0;
    private static final double TAG_HEIGHT = 29.5;

    @Config
    public static class TrackingParams {
        // Camera
        public static double CAMERA_HEIGHT = 14.8;
        public static double CAMERA_ANGLE = 7.9;

        // Search
        public static double SEARCH_SPEED = 0.5;
        public static double SEARCH_TIMEOUT = 0.5;

        // PID gains - REDUCED to prevent oscillation
        public static double KP = 0.018;            // Much lower P gain
        public static double KI = 0.0;              // No integral initially
        public static double KD = 0.012;            // Higher D for damping

        // Dead zones - CRITICAL for stability
        public static double HEADING_DEAD_ZONE = 1.0;     // Stop if within 1 degree
        public static double HEADING_SLOW_ZONE = 3.0;     // Slow down within 3 degrees
        public static double VELOCITY_THRESHOLD = 2.0;    // Stop if error rate is low

        // Speed limits
        public static double MAX_TURN_SPEED = 0.5;        // Reduced max speed
        public static double MIN_TURN_SPEED = 0.1;       // Lower minimum
        public static double SLOW_ZONE_SPEED = 0.08;      // Speed in slow zone

        // Stability detection
        public static double STABLE_TIME = 0.3;           // Time to be considered stable
        public static double STABLE_ERROR = 1.5;          // Error threshold for stability

        // Distance
        public static double KP_DISTANCE = 0.02;
        public static double DISTANCE_TOLERANCE = 2.0;
        public static double MAX_DRIVE_SPEED = 0.4;
    }

    // Tracking
    private double tx = 0;
    private double ty = 0;
    private double calculatedDistance = 0;
    private boolean hasTarget = false;

    // PID state
    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime stableTimer = new ElapsedTime();
    private ElapsedTime lostTargetTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double errorVelocity = 0;

    // Smoothing
    private double[] errorHistory = new double[5];
    private int historyIndex = 0;

    // State
    private enum State {
        SEARCHING,
        ALIGNING,
        STABLE,
        APPROACHING,
        COMPLETE
    }
    private State currentState = State.SEARCHING;

    // Statistics
    private int oscillationCount = 0;
    private double lastErrorSign = 0;

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("═══ STABLE TRACKER ═══");
        telemetry.addLine();
        telemetry.addLine("• No oscillation");
        telemetry.addLine("• Dead zone: " + TrackingParams.HEADING_DEAD_ZONE + "°");
        telemetry.addLine("• Target: Tag " + TARGET_TAG_ID);
        telemetry.addLine();
        telemetry.addLine(">>> Press START <<<");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        pidTimer.reset();

        while (opModeIsActive()) {

            updateTracking();
            handleStateTransitions();

            switch (currentState) {
                case SEARCHING:
                    executeSearch();
                    break;

                case ALIGNING:
                    executeStableAlignment();
                    break;

                case STABLE:
                    maintainStability();
                    break;

                case APPROACHING:
                    executeApproach();
                    break;

                case COMPLETE:
                    driveController.stopDrive();
                    break;
            }

            displayTelemetry();
            sendDashboardData();

            sleep(10); // 100Hz
        }

        driveController.stopDrive();
    }

    /**
     * Stable alignment without oscillation
     */
    private void executeStableAlignment() {
        if (!hasTarget) {
            return;
        }

        double headingError = tx;
        double absError = Math.abs(headingError);

        // Add to error history for smoothing
        errorHistory[historyIndex] = headingError;
        historyIndex = (historyIndex + 1) % errorHistory.length;

        // Calculate average error for smoothing
        double avgError = 0;
        for (double e : errorHistory) {
            avgError += e;
        }
        avgError /= errorHistory.length;

        // DEAD ZONE - Stop if close enough
        if (absError < TrackingParams.HEADING_DEAD_ZONE) {
            driveController.stopDrive();
            integralSum = 0;

            // Check if stable
            if (absError < TrackingParams.STABLE_ERROR) {
                if (stableTimer.seconds() > TrackingParams.STABLE_TIME) {
                    currentState = State.STABLE;
                }
            } else {
                stableTimer.reset();
            }
            return;
        }

        stableTimer.reset(); // Not stable if we're here

        // Detect oscillation
        double errorSign = Math.signum(headingError);
        if (lastErrorSign != 0 && errorSign != lastErrorSign) {
            oscillationCount++;
            // Reduce gains if oscillating
            if (oscillationCount > 2) {
                integralSum *= 0.5; // Reduce integral
            }
        }
        lastErrorSign = errorSign;

        // PID Calculation
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt > 0) {
            // Use smoothed error for P term
            double pTerm = TrackingParams.KP * avgError;

            // Only accumulate integral outside dead zone
            if (absError > TrackingParams.HEADING_DEAD_ZONE) {
                integralSum += headingError * dt;
                integralSum = Range.clip(integralSum, -10, 10);
            }
            double iTerm = TrackingParams.KI * integralSum;

            // Derivative with filtering
            errorVelocity = (headingError - lastError) / dt;
            double dTerm = TrackingParams.KD * errorVelocity;
            lastError = headingError;

            // Combined output
            double turnPower = pTerm + iTerm + dTerm;

            // VELOCITY-BASED STOPPING
            // If error is small and we're not moving fast toward target, stop
            if (absError < TrackingParams.HEADING_SLOW_ZONE &&
                    Math.abs(errorVelocity) < TrackingParams.VELOCITY_THRESHOLD) {
                turnPower *= 0.5; // Reduce power in slow zone
            }

            // Speed zones
            if (absError < TrackingParams.HEADING_SLOW_ZONE) {
                // In slow zone - cap speed
                turnPower = Range.clip(turnPower,
                        -TrackingParams.SLOW_ZONE_SPEED,
                        TrackingParams.SLOW_ZONE_SPEED);
            } else {
                // Normal zone
                turnPower = Range.clip(turnPower,
                        -TrackingParams.MAX_TURN_SPEED,
                        TrackingParams.MAX_TURN_SPEED);
            }

            // Minimum power only if outside slow zone
            if (Math.abs(turnPower) < TrackingParams.MIN_TURN_SPEED &&
                    absError > TrackingParams.HEADING_SLOW_ZONE) {
                turnPower = Math.signum(turnPower) * TrackingParams.MIN_TURN_SPEED;
            }

            // Apply power
            driveController.tankDrive(turnPower, -turnPower);
        }
    }

    /**
     * Maintain stability when aligned
     */
    private void maintainStability() {
        if (!hasTarget) {
            currentState = State.SEARCHING;
            return;
        }

        double headingError = Math.abs(tx);

        // Check if we're still stable
        if (headingError < TrackingParams.HEADING_DEAD_ZONE) {
            // Still stable - hold position
            driveController.stopDrive();

            // Move to approach after being stable
            if (stableTimer.seconds() > TrackingParams.STABLE_TIME * 2) {
                currentState = State.APPROACHING;
            }
        } else if (headingError > TrackingParams.STABLE_ERROR) {
            // Lost stability - realign
            currentState = State.ALIGNING;
            stableTimer.reset();
            oscillationCount = 0;
        } else {
            // Small correction without full PID
            double correction = TrackingParams.KP * tx * 0.5;
            correction = Range.clip(correction,
                    -TrackingParams.SLOW_ZONE_SPEED,
                    TrackingParams.SLOW_ZONE_SPEED);
            driveController.tankDrive(correction, -correction);
        }
    }

    /**
     * Handle state transitions
     */
    private void handleStateTransitions() {
        if (!hasTarget && currentState != State.SEARCHING) {
            if (lostTargetTimer.seconds() > TrackingParams.SEARCH_TIMEOUT) {
                currentState = State.SEARCHING;
                resetPID();
            }
        } else if (hasTarget) {
            lostTargetTimer.reset();

            if (currentState == State.SEARCHING) {
                currentState = State.ALIGNING;
                resetPID();
            }
        }
    }

    /**
     * Reset PID state
     */
    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        errorVelocity = 0;
        oscillationCount = 0;
        lastErrorSign = 0;
        stableTimer.reset();
        pidTimer.reset();

        // Clear error history
        for (int i = 0; i < errorHistory.length; i++) {
            errorHistory[i] = 0;
        }
    }

    /**
     * Search for tag
     */
    private void executeSearch() {
        driveController.tankDrive(-TrackingParams.SEARCH_SPEED, TrackingParams.SEARCH_SPEED);

        if (hasTarget) {
            driveController.stopDrive();
            sleep(50);
        }
    }

    /**
     * Approach target distance
     */
    private void executeApproach() {
        if (!hasTarget) {
            return;
        }

        double headingError = tx;
        double distanceError = TARGET_DISTANCE - calculatedDistance;

        // Maintain heading
        if (Math.abs(headingError) > TrackingParams.STABLE_ERROR * 2) {
            currentState = State.ALIGNING;
            return;
        }

        if (Math.abs(distanceError) < TrackingParams.DISTANCE_TOLERANCE) {
            driveController.stopDrive();
            currentState = State.COMPLETE;
            return;
        }

        // Distance control with gentle heading correction
        double drivePower = TrackingParams.KP_DISTANCE * distanceError;
        drivePower = Range.clip(drivePower, -TrackingParams.MAX_DRIVE_SPEED,
                TrackingParams.MAX_DRIVE_SPEED);

        // Very gentle heading correction
        double headingCorrection = TrackingParams.KP * headingError * 0.3;

        double leftPower = drivePower + headingCorrection;
        double rightPower = drivePower - headingCorrection;

        driveController.tankDrive(leftPower, rightPower);
    }

    /**
     * Update tracking
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
                        calculatedDistance = calculateDistance(ty);
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

    private double calculateDistance(double ty) {
        double totalAngle = TrackingParams.CAMERA_ANGLE + ty;
        double angleRadians = Math.toRadians(totalAngle);
        return Math.abs((TAG_HEIGHT - TrackingParams.CAMERA_HEIGHT) / Math.tan(angleRadians));
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
            // Continue without Limelight
        }
    }

    private void displayTelemetry() {
        telemetry.clear();
        telemetry.addLine("═══ STABLE TRACKING ═══");
        telemetry.addLine();

        telemetry.addData("State", currentState);
        telemetry.addData("TX", "%.2f°", tx);

        if (currentState == State.ALIGNING) {
            telemetry.addData("Error Velocity", "%.2f°/s", errorVelocity);
            telemetry.addData("Oscillations", oscillationCount);

            double absError = Math.abs(tx);
            if (absError < TrackingParams.HEADING_DEAD_ZONE) {
                telemetry.addLine(">>> IN DEAD ZONE <<<");
            } else if (absError < TrackingParams.HEADING_SLOW_ZONE) {
                telemetry.addLine(">> SLOW ZONE <<");
            }
        } else if (currentState == State.STABLE) {
            telemetry.addLine("✓✓✓ STABLE ✓✓✓");
            telemetry.addData("Stable Time", "%.1fs", stableTimer.seconds());
        }

        telemetry.update();
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("State", currentState.toString());
        packet.put("TX", tx);
        packet.put("Error_Velocity", errorVelocity);
        packet.put("Oscillations", oscillationCount);
        dashboard.sendTelemetryPacket(packet);
    }
}
