package org.firstinspires.ftc.teamcode.champion.Auton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Simple Limelight Vision for FTC
 * Uses YawPitchRollAngles from Pose3D
 */
public class
SimpleLimelightVision {

    private final Limelight3A limelight;
    private static final double METERS_TO_INCHES = 39.3701;

    public SimpleLimelightVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    /**
     * Get X position (inches from field center)
     */
    public double getX() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                return botpose.getPosition().x * METERS_TO_INCHES;
            }
        }
        return 0;
    }

    /**
     * Get Y position (inches from field center)
     */
    public double getY() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                return botpose.getPosition().y * METERS_TO_INCHES;
            }
        }
        return 0;
    }

    /**
     * Get heading (degrees, -180 to +180)
     * Extracts yaw from YawPitchRollAngles
     */
    public double getHeading() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Get orientation as YawPitchRollAngles
                YawPitchRollAngles orientation = botpose.getOrientation();

                // Extract yaw in degrees
                double yaw = orientation.getYaw(AngleUnit.DEGREES);

                // Normalize to -180 to +180
                while (yaw < 180 && yaw>90) yaw = -(180-yaw);
                while (yaw > -180 && yaw<-90) yaw += 180;

                return yaw;
            }
        }
        return 0;
    }

    /**
     * Get raw yaw for debugging (before normalization)
     */
    public double getRawYaw() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                YawPitchRollAngles orientation = botpose.getOrientation();
                return orientation.getYaw(AngleUnit.DEGREES);
            }
        }
        return 0;
    }

    /**
     * Get pitch angle (for debugging)
     */
    public double getPitch() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                YawPitchRollAngles orientation = botpose.getOrientation();
                return orientation.getPitch(AngleUnit.DEGREES);
            }
        }
        return 0;
    }

    /**
     * Get roll angle (for debugging)
     */
    public double getRoll() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                YawPitchRollAngles orientation = botpose.getOrientation();
                return orientation.getRoll(AngleUnit.DEGREES);
            }
        }
        return 0;
    }

    /**
     * Check if we have valid vision data
     */
    public boolean hasValidData() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose() != null;
        }
        return false;
    }

    /**
     * Get number of visible AprilTags
     */
    public int getTagCount() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults().size();
        }
        return 0;
    }

    /**
     * Get complete pose info string for debugging
     */
    public String getDebugInfo() {
        if (!hasValidData()) {
            return "NO DATA";
        }

        return String.format("X: %.1f\" Y: %.1f\" H: %.1f° (Raw: %.1f°) Tags: %d",
                getX(), getY(), getHeading(), getRawYaw(), getTagCount());
    }

    /**
     * Shutdown
     */
    public void shutdown() {
        limelight.stop();
    }

    // ========== ROBOT CENTER POSITION METHODS ==========

    /**
     * Get robot center X position (inches from field center)
     * Accounts for Limelight offset from robot center
     *
     * @param limelightOffsetY Distance from robot center to Limelight (positive = Limelight is forward)
     */
    public double getRobotCenterX(double limelightOffsetY) {
        if (!hasValidData()) return 0;

        // Get Limelight position and heading
        double llX = getX();
        double llY = getY();
        double heading = Math.toRadians(getHeading());

        // Calculate robot center position
        // Move backwards from Limelight by offsetY amount along robot's heading direction
        double centerX = llX - limelightOffsetY * Math.sin(heading);

        return centerX;
    }

    /**
     * Get robot center Y position (inches from field center)
     * Accounts for Limelight offset from robot center
     *
     * @param limelightOffsetY Distance from robot center to Limelight (positive = Limelight is forward)
     */
    public double getRobotCenterY(double limelightOffsetY) {
        if (!hasValidData()) return 0;

        // Get Limelight position and heading
        double llX = getX();
        double llY = getY();
        double heading = Math.toRadians(getHeading());

        // Calculate robot center position
        // Move backwards from Limelight by offsetY amount along robot's heading direction
        double centerY = llY - limelightOffsetY * Math.cos(heading);

        return centerY;
    }

    /**
     * Calculate heading from robot center to a target position
     * Returns absolute heading (0° = toward RED wall)
     *
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @param limelightOffsetY Limelight offset from robot center
     * @return Heading to target in degrees (-180 to +180)
     */
    public double getHeadingToTarget(double targetX, double targetY, double limelightOffsetY) {
        if (!hasValidData()) return 0;

        // Get robot center position
        double centerX = getRobotCenterX(limelightOffsetY);
        double centerY = getRobotCenterY(limelightOffsetY);

        // Calculate vector from robot center to target
        double deltaX = targetX - centerX;
        double deltaY = targetY - centerY;

        // Calculate angle to target
        // atan2(deltaX, deltaY) because in field coordinates:
        // - Y axis points toward RED wall (0°)
        // - X axis points right
        double angleToTarget = Math.toDegrees(Math.atan2(deltaX, deltaY));

        // Normalize to -180 to +180
        while (angleToTarget > 180) angleToTarget -= 360;
        while (angleToTarget < -180) angleToTarget += 360;

        return angleToTarget;
    }

    /**
     * Calculate heading ERROR (how much to turn to face target)
     * Positive = turn left (CCW), Negative = turn right (CW)
     *
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @param limelightOffsetY Limelight offset from robot center
     * @return Heading error in degrees (-180 to +180)
     */
    public double getHeadingError(double targetX, double targetY, double limelightOffsetY) {
        if (!hasValidData()) return 0;

        // Get current robot heading (absolute)
        double currentHeading = getHeading();

        // Get heading to target (absolute)
        double targetHeading = getHeadingToTarget(targetX, targetY, limelightOffsetY);

        // Calculate error
        double error = targetHeading - currentHeading;

        // Normalize to -180 to +180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }

    /**
     * Calculate distance from robot center to target
     *
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @param limelightOffsetY Limelight offset from robot center
     * @return Distance in inches
     */
    public double getDistanceToTarget(double targetX, double targetY, double limelightOffsetY) {
        if (!hasValidData()) return 0;

        // Get robot center position
        double centerX = getRobotCenterX(limelightOffsetY);
        double centerY = getRobotCenterY(limelightOffsetY);

        // Calculate distance
        double deltaX = targetX - centerX;
        double deltaY = targetY - centerY;

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Get complete targeting info string
     */
    public String getTargetingInfo(double targetX, double targetY, double limelightOffsetY) {
        if (!hasValidData()) {
            return "NO DATA";
        }

        return String.format("Distance: %.1f\" | Heading Error: %.1f° | Robot: (%.1f\", %.1f\")",
                getDistanceToTarget(targetX, targetY, limelightOffsetY),
                getHeadingError(targetX, targetY, limelightOffsetY),
                getRobotCenterX(limelightOffsetY),
                getRobotCenterY(limelightOffsetY));
    }
}