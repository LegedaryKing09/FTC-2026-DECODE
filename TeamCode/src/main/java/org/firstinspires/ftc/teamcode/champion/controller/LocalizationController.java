package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
public class LocalizationController {

    // ========== CONFIGURABLE APRILTAG PARAMETERS ==========
    // Change these values to match your AprilTag setup
    public static int TARGET_APRILTAG_ID = 20;
    public static double APRILTAG_X = 0.0;  // X coordinate of AprilTag in inches
    public static double APRILTAG_Y = 0.0;  // Y coordinate of AprilTag in inches

    // ========== LIMELIGHT CONFIGURATION ==========
    public static String LIMELIGHT_NAME = "limelight";
    public static int PIPELINE_INDEX = 0;  // Pipeline for AprilTag detection

    // Conversion factor
    private static final double METERS_TO_INCHES = 39.3701;

    private final Limelight3A limelight;
    private boolean isInitialized = false;

    /**
     * Result class containing robot position
     */
    public static class RobotPosition {
        public final double x;
        public final double y;
        public final boolean isValid;

        public RobotPosition(double x, double y, boolean isValid) {
            this.x = x;
            this.y = y;
            this.isValid = isValid;
        }

        /**
         * Returns an invalid position (used when no AprilTag is detected)
         */
        public static RobotPosition invalid() {
            return new RobotPosition(0, 0, false);
        }

        @Override
        public String toString() {
            if (!isValid) {
                return "Position: INVALID (no tag detected)";
            }
            return String.format("Position: (%.2f, %.2f) inches", x, y);
        }
    }

    /**
     * Constructor - initializes Limelight 3A
     * @param hardwareMap The hardware map from the OpMode
     */
    public LocalizationController(HardwareMap hardwareMap) {
        Limelight3A tempLimelight = null;
        try {
            tempLimelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            tempLimelight.pipelineSwitch(PIPELINE_INDEX);
            tempLimelight.start();
            isInitialized = true;
        } catch (Exception e) {
            isInitialized = false;
        }
        this.limelight = tempLimelight;
    }

    /**
     * Get the robot's position based on the configured AprilTag
     *
     * @return RobotPosition containing x, y coordinates and validity flag
     */
    public RobotPosition getRobotPosition() {
        if (!isInitialized || limelight == null) {
            return RobotPosition.invalid();
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return RobotPosition.invalid();
        }

        // Look for the specific AprilTag
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                // Get the robot's pose relative to the tag
                Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();

                if (robotPoseTargetSpace != null) {
                    // Robot position relative to the tag (in meters, convert to inches)
                    double relativeX = robotPoseTargetSpace.getPosition().x * METERS_TO_INCHES;
                    double relativeY = robotPoseTargetSpace.getPosition().y * METERS_TO_INCHES;

                    // Calculate absolute robot position by adding tag's field position
                    double robotX = APRILTAG_X + relativeX;
                    double robotY = APRILTAG_Y + relativeY;

                    return new RobotPosition(robotX, robotY, true);
                }
            }
        }

        return RobotPosition.invalid();
    }

    /**
     * Get robot X coordinate only
     * @return X coordinate in inches, or 0 if no valid reading
     */
    public double getX() {
        RobotPosition pos = getRobotPosition();
        return pos.isValid ? pos.x : 0;
    }

    /**
     * Get robot Y coordinate only
     * @return Y coordinate in inches, or 0 if no valid reading
     */
    public double getY() {
        RobotPosition pos = getRobotPosition();
        return pos.isValid ? pos.y : 0;
    }

    /**
     * Check if the target AprilTag is currently visible
     * @return true if the configured AprilTag is detected
     */
    public boolean isTagVisible() {
        if (!isInitialized || limelight == null) {
            return false;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                return true;
            }
        }
        return false;
    }

    /**
     * Get the number of AprilTags currently visible
     * @return Count of visible AprilTags
     */
    public int getVisibleTagCount() {
        if (!isInitialized || limelight == null) {
            return 0;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0;
        }

        return result.getFiducialResults().size();
    }

    /**
     * Update the target AprilTag ID at runtime
     * @param tagId The new AprilTag ID to track
     */
    public void setTargetAprilTagId(int tagId) {
        TARGET_APRILTAG_ID = tagId;
    }

    /**
     * Update the AprilTag's field position at runtime
     * @param x X coordinate of AprilTag in inches
     * @param y Y coordinate of AprilTag in inches
     */
    public void setAprilTagPosition(double x, double y) {
        APRILTAG_X = x;
        APRILTAG_Y = y;
    }

    /**
     * Switch the Limelight pipeline
     * @param pipelineIndex The pipeline index to switch to
     */
    public void switchPipeline(int pipelineIndex) {
        if (isInitialized && limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    /**
     * Check if the Limelight is properly initialized
     * @return true if initialized successfully
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Get debug information string
     * @return Formatted debug string with position and status
     */
    public String getDebugInfo() {
        if (!isInitialized) {
            return "Limelight NOT INITIALIZED";
        }

        RobotPosition pos = getRobotPosition();
        if (!pos.isValid) {
            return String.format("Tag %d: NOT VISIBLE | Tags seen: %d",
                    TARGET_APRILTAG_ID, getVisibleTagCount());
        }

        return String.format("Tag %d: VISIBLE | Robot: (%.1f, %.1f) in | Tags: %d",
                TARGET_APRILTAG_ID, pos.x, pos.y, getVisibleTagCount());
    }

    /**
     * Shutdown the Limelight
     */
    public void shutdown() {
        if (isInitialized && limelight != null) {
            limelight.stop();
        }
    }
}
