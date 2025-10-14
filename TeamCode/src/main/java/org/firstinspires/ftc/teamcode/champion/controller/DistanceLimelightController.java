package org.firstinspires.ftc.teamcode.champion.controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.config.Config;

import java.util.List;

/**
 * Enhanced Limelight Controller with Distance Calculation
 *
 * Extends LimelightAlignmentController to add distance measurement
 * using the ty angle (vertical offset) from the Limelight.
 *
 * DISTANCE FORMULA:
 * distance = (target_height - camera_height) / tan(camera_angle + ty)
 *
 * CALIBRATION:
 * Adjust these values in FTC Dashboard for your robot:
 * - CAMERA_HEIGHT_INCHES: Height of Limelight lens from ground
 * - CAMERA_ANGLE_DEGREES: Upward tilt angle of camera
 * - TARGET_HEIGHT_INCHES: Height of AprilTag center from ground
 */
@Config
public class DistanceLimelightController extends LimelightAlignmentController {

    @Config
    public static class DistanceParams {
        // Camera physical setup (ADJUST FOR YOUR ROBOT)
        public static double CAMERA_HEIGHT_INCHES = 12.0;     // Height of Limelight from ground
        public static double CAMERA_ANGLE_DEGREES = 15.0;     // Upward tilt angle of camera
        public static double TARGET_HEIGHT_INCHES = 29.5;     // Height of AprilTag center

        // Distance filtering
        public static double DISTANCE_FILTER_ALPHA = 0.7;     // Smoothing factor (0-1)
        public static double MAX_REASONABLE_DISTANCE = 200.0; // Max distance in inches
        public static double MIN_REASONABLE_DISTANCE = 10.0;  // Min distance in inches
    }

    private final Limelight3A limelight;
    private final SixWheelDriveController driveController;
    private final LinearOpMode opMode;

    private double currentDistance = 0;
    private double filteredDistance = 0;
    private double ty = 0;  // Vertical offset angle from Limelight
    private boolean hasValidDistance = false;
    private int targetTagId = 20;  // Track target tag locally

    /**
     * Constructor
     */
    public DistanceLimelightController(LinearOpMode opMode, SixWheelDriveController driveController) throws Exception {
        super(opMode);
        this.opMode = opMode;
        this.driveController = driveController;

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            throw new Exception("Failed to initialize Limelight for distance: " + e.getMessage());
        }
    }

    /**
     * Override setTargetTag to track it locally
     */
    @Override
    public void setTargetTag(int tagId) {
        super.setTargetTag(tagId);
        this.targetTagId = tagId;
    }

    /**
     * Calculate distance to target using Limelight ty angle
     * Formula: distance = (h2 - h1) / tan(a1 + a2)
     * Where:
     *   h2 = target height
     *   h1 = camera height
     *   a1 = camera mounting angle
     *   a2 = ty angle from Limelight
     */
    public double calculateDistance() {
        if (!hasTarget()) {
            hasValidDistance = false;
            return -1;
        }

        try {
            // Get ty angle (vertical offset) from Limelight
            updateTyAngle();

            // Calculate distance using trigonometry
            double angleToTarget = DistanceParams.CAMERA_ANGLE_DEGREES + ty;
            double angleRadians = Math.toRadians(angleToTarget);

            double heightDifference = DistanceParams.TARGET_HEIGHT_INCHES -
                    DistanceParams.CAMERA_HEIGHT_INCHES;

            currentDistance = heightDifference / Math.tan(angleRadians);

            // Validate distance is reasonable
            if (currentDistance < DistanceParams.MIN_REASONABLE_DISTANCE ||
                    currentDistance > DistanceParams.MAX_REASONABLE_DISTANCE) {
                hasValidDistance = false;
                return filteredDistance; // Return last good value
            }

            // Apply smoothing filter
            if (filteredDistance == 0 || !hasValidDistance) {
                // First reading or recovering from invalid
                filteredDistance = currentDistance;
            } else {
                // Exponential moving average
                filteredDistance = (DistanceParams.DISTANCE_FILTER_ALPHA * currentDistance) +
                        ((1 - DistanceParams.DISTANCE_FILTER_ALPHA) * filteredDistance);
            }

            hasValidDistance = true;
            return filteredDistance;

        } catch (Exception e) {
            hasValidDistance = false;
            return filteredDistance; // Return last good value
        }
    }

    /**
     * Get the current distance to target (filtered)
     */
    public double getDistanceToTarget() {
        calculateDistance(); // Update distance
        return hasValidDistance ? filteredDistance : -1;
    }

    /**
     * Get raw unfiltered distance
     */
    public double getRawDistance() {
        return currentDistance;
    }

    /**
     * Check if we have a valid distance measurement
     */
    public boolean hasValidDistance() {
        return hasValidDistance;
    }

    /**
     * Get the ty angle (vertical offset) from Limelight
     */
    public double getTyAngle() {
        return ty;
    }

    /**
     * Update ty angle from Limelight data
     */
    private void updateTyAngle() {
        if (limelight == null) {
            ty = 0;
            return;
        }

        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == getTargetTagId()) {
                        ty = fiducial.getTargetYDegrees();
                        break;
                    }
                }
            }
        } catch (Exception e) {
            ty = 0;
        }
    }

    /**
     * Override to expose target tag ID for distance calculation
     * Make sure to add this getter to LimelightAlignmentController:
     * public int getTargetTagId() { return targetTagId; }
     */

    /**
     * Display distance telemetry
     */
    public void displayDistanceTelemetry() {
        if (hasValidDistance) {
            opMode.telemetry.addData("Distance", "%.1f inches", filteredDistance);
            opMode.telemetry.addData("Distance (raw)", "%.1f inches", currentDistance);
            opMode.telemetry.addData("TY Angle", "%.2f°", ty);
        } else {
            opMode.telemetry.addData("Distance", "INVALID");
        }

        // Show calculation parameters
        opMode.telemetry.addLine();
        opMode.telemetry.addLine("── DISTANCE CALIBRATION ──");
        opMode.telemetry.addData("Camera Height", "%.1f in", DistanceParams.CAMERA_HEIGHT_INCHES);
        opMode.telemetry.addData("Camera Angle", "%.1f°", DistanceParams.CAMERA_ANGLE_DEGREES);
        opMode.telemetry.addData("Target Height", "%.1f in", DistanceParams.TARGET_HEIGHT_INCHES);
    }

    /**
     * Reset distance tracking
     */
    public void resetDistance() {
        currentDistance = 0;
        filteredDistance = 0;
        hasValidDistance = false;
        ty = 0;
    }

    /**
     * Get recommended shooting power based on distance
     * This is a helper method for the shooter controller
     */
    public double getRecommendedPower(double distanceThreshold, double farPower, double closePower) {
        double distance = getDistanceToTarget();
        if (distance <= 0) {
            return farPower; // Default to far if can't measure
        }

        return distance > distanceThreshold ? farPower : closePower;
    }

    /**
     * Get recommended RPM based on distance
     */
    public double getRecommendedRPM(double distanceThreshold, double farRPM, double closeRPM) {
        double distance = getDistanceToTarget();
        if (distance <= 0) {
            return farRPM; // Default to far if can't measure
        }

        return distance > distanceThreshold ? farRPM : closeRPM;
    }
}
