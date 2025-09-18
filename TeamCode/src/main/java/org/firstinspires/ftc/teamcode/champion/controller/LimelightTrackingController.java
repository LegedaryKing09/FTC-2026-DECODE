package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

@Config
public class LimelightTrackingController {

    @Config
    public static class TrackingParams {
        // Camera
        public static double CAMERA_HEIGHT = 14.8;
        public static double CAMERA_ANGLE = 7.9;
        public static double TAG_HEIGHT = 29.5;

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

    public enum TrackingState {
        SEARCHING,
        ALIGNING,
        STABLE,
        APPROACHING,
        COMPLETE,
        NO_TARGET,
        ERROR
    }

    public static class TrackingResult {
        public TrackingState state;
        public double leftPower;
        public double rightPower;
        public double tx;
        public double distance;
        public boolean hasTarget;
        public double errorVelocity;
        public int oscillationCount;
        public double stableTime;

        public TrackingResult(TrackingState state, double leftPower, double rightPower) {
            this.state = state;
            this.leftPower = leftPower;
            this.rightPower = rightPower;
        }
    }

    private final Limelight3A limelight;

    // Tracking state
    private int targetTagId;
    private double targetDistance;
    private TrackingState currentState = TrackingState.SEARCHING;

    // Tracking data
    private double tx = 0;
    private double calculatedDistance = 0;
    private boolean hasTarget = false;

    // PID state
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime stableTimer = new ElapsedTime();
    private final ElapsedTime lostTargetTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private double errorVelocity = 0;

    // Smoothing
    private final double[] errorHistory = new double[5];
    private int historyIndex = 0;

    // Statistics
    private int oscillationCount = 0;
    private double lastErrorSign = 0;

    public LimelightTrackingController(LinearOpMode opMode) {

        try {
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize Limelight: " + e.getMessage());
        }

        pidTimer.reset();
    }

    /**
     * Track and align to a specific AprilTag
     * @param targetTagId The AprilTag ID to track
     * @param targetDistance Desired distance from the tag (inches)
     * @return TrackingResult with motor powers and status
     */
    public TrackingResult trackTarget(int targetTagId, double targetDistance) {
        this.targetTagId = targetTagId;
        this.targetDistance = targetDistance;

        updateTracking();
        handleStateTransitions();

        TrackingResult result = new TrackingResult(currentState, 0, 0);
        result.tx = tx;
        result.distance = calculatedDistance;
        result.hasTarget = hasTarget;
        result.errorVelocity = errorVelocity;
        result.oscillationCount = oscillationCount;
        result.stableTime = stableTimer.seconds();

        switch (currentState) {
            case SEARCHING:
                executeSearch(result);
                break;
            case ALIGNING:
                executeStableAlignment(result);
                break;
            case STABLE:
                maintainStability(result);
                break;
            case APPROACHING:
                executeApproach(result);
                break;
            case COMPLETE:
            case NO_TARGET:
            case ERROR:
                result.leftPower = 0;
                result.rightPower = 0;
                break;
        }

        return result;
    }

    /**
     * Simple alignment function - just align to target, don't approach
     */
    public TrackingResult alignToTarget(int targetTagId) {
        this.targetTagId = targetTagId;
        this.targetDistance = 0; // Not used for alignment only

        updateTracking();

        TrackingResult result = new TrackingResult(currentState, 0, 0);
        result.tx = tx;
        result.distance = calculatedDistance;
        result.hasTarget = hasTarget;
        result.errorVelocity = errorVelocity;
        result.oscillationCount = oscillationCount;
        result.stableTime = stableTimer.seconds();

        if (!hasTarget) {
            executeSearch(result);
            return result;
        }

        // Only do alignment, skip approach
        if (Math.abs(tx) < TrackingParams.HEADING_DEAD_ZONE) {
            result.state = TrackingState.STABLE;
            result.leftPower = 0;
            result.rightPower = 0;
        } else {
            result.state = TrackingState.ALIGNING;
            executeStableAlignment(result);
        }

        return result;
    }

    /**
     * Reset the controller state
     */
    public void reset() {
        currentState = TrackingState.SEARCHING;
        resetPID();
    }

    /**
     * Check if tracking is complete
     */
    public boolean isTrackingComplete() {
        return currentState == TrackingState.COMPLETE;
    }

    /**
     * Check if target is aligned (within dead zone)
     */
    public boolean isAligned() {
        return hasTarget && Math.abs(tx) < TrackingParams.HEADING_DEAD_ZONE;
    }

    /**
     * Get current tracking state
     */
    public TrackingState getCurrentState() {
        return currentState;
    }

    /**
     * Get current target X offset
     */
    public double getCurrentTX() {
        return tx;
    }

    /**
     * Get calculated distance to target
     */
    public double getCurrentDistance() {
        return calculatedDistance;
    }

    /**
     * Check if target is visible
     */
    public boolean hasTarget() {
        return hasTarget;
    }

    private void executeSearch(TrackingResult result) {
        result.leftPower = -TrackingParams.SEARCH_SPEED;
        result.rightPower = TrackingParams.SEARCH_SPEED;

        if (hasTarget) {
            result.leftPower = 0;
            result.rightPower = 0;
        }
    }

    private void executeStableAlignment(TrackingResult result) {
        if (!hasTarget) {
            result.leftPower = 0;
            result.rightPower = 0;
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
            result.leftPower = 0;
            result.rightPower = 0;
            integralSum = 0;

            // Check if stable
            if (absError < TrackingParams.STABLE_ERROR) {
                if (stableTimer.seconds() > TrackingParams.STABLE_TIME) {
                    currentState = TrackingState.STABLE;
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
            if (absError < TrackingParams.HEADING_SLOW_ZONE &&
                    Math.abs(errorVelocity) < TrackingParams.VELOCITY_THRESHOLD) {
                turnPower *= 0.5;
            }

            // Speed zones
            if (absError < TrackingParams.HEADING_SLOW_ZONE) {
                turnPower = Range.clip(turnPower,
                        -TrackingParams.SLOW_ZONE_SPEED,
                        TrackingParams.SLOW_ZONE_SPEED);
            } else {
                turnPower = Range.clip(turnPower,
                        -TrackingParams.MAX_TURN_SPEED,
                        TrackingParams.MAX_TURN_SPEED);
            }

            // Minimum power only if outside slow zone
            if (Math.abs(turnPower) < TrackingParams.MIN_TURN_SPEED &&
                    absError > TrackingParams.HEADING_SLOW_ZONE) {
                turnPower = Math.signum(turnPower) * TrackingParams.MIN_TURN_SPEED;
            }

            result.leftPower = turnPower;
            result.rightPower = -turnPower;
        }
    }

    private void maintainStability(TrackingResult result) {
        if (!hasTarget) {
            currentState = TrackingState.SEARCHING;
            result.leftPower = 0;
            result.rightPower = 0;
            return;
        }

        double headingError = Math.abs(tx);

        if (headingError < TrackingParams.HEADING_DEAD_ZONE) {
            result.leftPower = 0;
            result.rightPower = 0;

            // Move to approach after being stable (if target distance is set)
            if (targetDistance > 0 && stableTimer.seconds() > TrackingParams.STABLE_TIME * 2) {
                currentState = TrackingState.APPROACHING;
            }
        } else if (headingError > TrackingParams.STABLE_ERROR) {
            currentState = TrackingState.ALIGNING;
            stableTimer.reset();
            oscillationCount = 0;
            result.leftPower = 0;
            result.rightPower = 0;
        } else {
            // Small correction without full PID
            double correction = TrackingParams.KP * tx * 0.5;
            correction = Range.clip(correction,
                    -TrackingParams.SLOW_ZONE_SPEED,
                    TrackingParams.SLOW_ZONE_SPEED);
            result.leftPower = correction;
            result.rightPower = -correction;
        }
    }

    private void executeApproach(TrackingResult result) {
        if (!hasTarget) {
            result.leftPower = 0;
            result.rightPower = 0;
            return;
        }

        double headingError = tx;
        double distanceError = targetDistance - calculatedDistance;

        // Maintain heading
        if (Math.abs(headingError) > TrackingParams.STABLE_ERROR * 2) {
            currentState = TrackingState.ALIGNING;
            result.leftPower = 0;
            result.rightPower = 0;
            return;
        }

        if (Math.abs(distanceError) < TrackingParams.DISTANCE_TOLERANCE) {
            currentState = TrackingState.COMPLETE;
            result.leftPower = 0;
            result.rightPower = 0;
            return;
        }

        // Distance control with gentle heading correction
        double drivePower = TrackingParams.KP_DISTANCE * distanceError;
        drivePower = Range.clip(drivePower, -TrackingParams.MAX_DRIVE_SPEED,
                TrackingParams.MAX_DRIVE_SPEED);

        // Very gentle heading correction
        double headingCorrection = TrackingParams.KP * headingError * 0.3;

        result.leftPower = drivePower + headingCorrection;
        result.rightPower = drivePower - headingCorrection;
    }

    private void handleStateTransitions() {
        if (!hasTarget && currentState != TrackingState.SEARCHING) {
            if (lostTargetTimer.seconds() > TrackingParams.SEARCH_TIMEOUT) {
                currentState = TrackingState.SEARCHING;
                resetPID();
            }
        } else if (hasTarget) {
            lostTargetTimer.reset();

            if (currentState == TrackingState.SEARCHING) {
                currentState = TrackingState.ALIGNING;
                resetPID();
            }
        }
    }

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
        Arrays.fill(errorHistory, 0);
    }

    private void updateTracking() {
        if (limelight == null) {
            hasTarget = false;
            currentState = TrackingState.ERROR;
            return;
        }

        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                hasTarget = false;

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetTagId) {
                        hasTarget = true;
                        tx = fiducial.getTargetXDegrees();
                        double ty = fiducial.getTargetYDegrees();
                        calculatedDistance = calculateDistance(ty);
                        break;
                    }
                }
            } else {
                hasTarget = false;
            }
        } catch (Exception e) {
            hasTarget = false;
            currentState = TrackingState.ERROR;
        }
    }

    private double calculateDistance(double ty) {
        double totalAngle = TrackingParams.CAMERA_ANGLE + ty;
        double angleRadians = Math.toRadians(totalAngle);
        return Math.abs((TrackingParams.TAG_HEIGHT - TrackingParams.CAMERA_HEIGHT) / Math.tan(angleRadians));
    }
}