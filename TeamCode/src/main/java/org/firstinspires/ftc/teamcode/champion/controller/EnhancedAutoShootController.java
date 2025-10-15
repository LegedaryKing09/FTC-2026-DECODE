package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class EnhancedAutoShootController {

    // ========== LIMELIGHT DISTANCE CALIBRATION ==========
    // Physical measurements (MEASURE THESE ON YOUR ROBOT!)
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 14.8;  // Distance from lens center to floor
    public static double GOAL_HEIGHT_INCHES = 29.5;            // Distance from target center to floor
    public static double LIMELIGHT_MOUNT_ANGLE_DEGREES = 19.7; // Camera tilt angle from vertical (TUNE THIS!)

    // ========== SIMPLE DISTANCE-TO-RPM MAPPING ==========
    public static double DISTANCE_THRESHOLD = 100;       // Distance threshold (inches)
    public static double RPM_CLOSE = 2600;               // RPM for distance < 100 inches
    public static double RPM_FAR = 3000;                 // RPM for distance >= 100 inches
    public static double MIN_DISTANCE = 12;              // Minimum shooting distance (inches)
    public static double MAX_DISTANCE = 200;             // Maximum shooting distance (inches)

    // ========== SHOOTING SEQUENCE PARAMETERS ==========
    public static int APRILTAG_ID = 20;                      // AprilTag to align to
    public static double ALIGNMENT_THRESHOLD = 3.0;          // Alignment error threshold (degrees)
    public static long ALIGNMENT_TIMEOUT = 3000;             // Max alignment time (ms)
    public static long RPM_TIMEOUT = 3000;                   // Max RPM spin-up time (ms)
    public static long SHOOT_DURATION = 1000;                // Transfer/intake duration (ms)
    public static long STABILITY_DELAY = 300;                // Stabilization delay (ms)
    public static double RPM_TOLERANCE = 100;                // RPM error tolerance

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final ShooterController shooterController;
    private final IntakeController intakeController;
    private final TransferController transferController;
    private final LimelightAlignmentController limelightController;
    private Limelight3A limelight;

    private boolean isAutoShooting = false;
    private int shotsCompleted = 0;
    private double lastShotTime = 0;
    private double lastTargetDistance = 0;
    private double lastCalculatedRPM = 0;
    private final ElapsedTime sessionTimer = new ElapsedTime();

    // Sequence status
    private String currentStatus = "IDLE";

    public EnhancedAutoShootController(LinearOpMode opMode,
                                       SixWheelDriveController driveController,
                                       ShooterController shooterController,
                                       IntakeController intakeController,
                                       TransferController transferController,
                                       LimelightAlignmentController limelightController) {
        this.opMode = opMode;
        this.driveController = driveController;
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.transferController = transferController;
        this.limelightController = limelightController;

        // Get Limelight reference from hardware map
        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            this.limelight = null;
            if (opMode.telemetry != null) {
                opMode.telemetry.addData("Warning", "Limelight not found for distance calculation");
            }
        }
    }

    /**
     * Main method to execute the complete auto-shoot sequence with distance-based RPM
     */
    public void executeDistanceBasedAutoShoot() {
        if (isAutoShooting) {
            opMode.telemetry.addLine("⚠️ Auto-shoot already in progress!");
            return;
        }

        isAutoShooting = true;

        new Thread(() -> {
            try {
                // Step 1: Find distance to AprilTag
                currentStatus = "FINDING DISTANCE";
                double distance = getDistanceToTarget(APRILTAG_ID);

                if (distance < 0) {
                    currentStatus = "TARGET NOT FOUND";
                    opMode.telemetry.addLine("❌ Cannot find target AprilTag!");
                    opMode.telemetry.update();
                    Thread.sleep(2000);
                    return;
                }

                lastTargetDistance = distance;

                // Step 2: Calculate required RPM based on distance
                currentStatus = "CALCULATING RPM";
                double targetRPM = calculateRPMFromDistance(distance);
                lastCalculatedRPM = targetRPM;

                opMode.telemetry.addLine("=== AUTO-SHOOT SEQUENCE ===");
                opMode.telemetry.addData("Distance", "%.1f inches", distance);
                opMode.telemetry.addData("Target RPM", "%.0f", targetRPM);
                opMode.telemetry.update();
                Thread.sleep(500);

                // Step 3: Stop movement and start shooter
                currentStatus = "SPINNING UP";
                driveController.stopDrive();
                shooterController.setShooterRPM(targetRPM);

                // Wait for shooter to reach target RPM
                long rpmStartTime = System.currentTimeMillis();
                while (opMode.opModeIsActive() &&
                        !shooterController.isAtTargetRPM() &&
                        (System.currentTimeMillis() - rpmStartTime) < RPM_TIMEOUT) {

                    opMode.telemetry.addLine("⚙️ Spinning up shooter...");
                    opMode.telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
                    opMode.telemetry.addData("Target RPM", "%.0f", targetRPM);
                    opMode.telemetry.addData("Error", "%.0f RPM", shooterController.getRPMError());
                    opMode.telemetry.update();

                    Thread.sleep(20);
                }

                // Step 4: Align to target
                currentStatus = "ALIGNING";
                if (limelightController != null) {
                    limelightController.startAlignment();

                    long alignStartTime = System.currentTimeMillis();
                    boolean alignmentAchieved = false;

                    while (opMode.opModeIsActive() &&
                            (System.currentTimeMillis() - alignStartTime) < ALIGNMENT_TIMEOUT) {

                        limelightController.align(APRILTAG_ID);

                        double alignmentError = limelightController.getTargetError();

                        opMode.telemetry.addLine("🎯 Aligning to target...");
                        opMode.telemetry.addData("Alignment Error", "%.2f°", alignmentError);
                        opMode.telemetry.addData("Threshold", "%.2f°", ALIGNMENT_THRESHOLD);
                        opMode.telemetry.addData("Has Target", limelightController.hasTarget() ? "YES" : "NO");
                        opMode.telemetry.update();

                        // Check if aligned within threshold
                        if (limelightController.hasTarget() &&
                                alignmentError <= ALIGNMENT_THRESHOLD) {
                            alignmentAchieved = true;
                            break;
                        }

                        Thread.sleep(20);
                    }

                    // Stop alignment
                    limelightController.stopAlignment();
                    driveController.stopDrive();

                    if (!alignmentAchieved) {
                        opMode.telemetry.addLine("⚠️ Alignment timeout - shooting anyway");
                        opMode.telemetry.update();
                    }

                    // Stabilization delay
                    Thread.sleep(STABILITY_DELAY);
                }

                // Step 5: Execute shot
                currentStatus = "SHOOTING";

                // Verify RPM is still good
                if (shooterController.isAtTargetRPM() ||
                        Math.abs(shooterController.getRPMError()) < RPM_TOLERANCE) {

                    opMode.telemetry.addLine("🔥 FIRING!");
                    opMode.telemetry.update();

                    // Run transfer and intake to shoot
                    transferController.transferFull();
                    intakeController.intakeFull();
                    Thread.sleep(SHOOT_DURATION);

                    // Stop transfer and intake
                    transferController.transferStop();
                    intakeController.intakeStop();

                    shotsCompleted++;
                    lastShotTime = sessionTimer.seconds();

                    currentStatus = "SHOT COMPLETE";

                    opMode.telemetry.clear();
                    opMode.telemetry.addLine("=== ✅ SHOT COMPLETE ===");
                    opMode.telemetry.addData("Shot #", shotsCompleted);
                    opMode.telemetry.addData("Distance", "%.1f in", lastTargetDistance);
                    opMode.telemetry.addData("RPM Used", "%.0f", lastCalculatedRPM);
                    opMode.telemetry.update();

                } else {
                    currentStatus = "RPM NOT READY";
                    opMode.telemetry.addLine("❌ Shooter not at RPM - shot aborted");
                    opMode.telemetry.update();
                }

                // Keep shooter running for potential next shot

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentStatus = "INTERRUPTED";
            } catch (Exception e) {
                currentStatus = "ERROR: " + e.getMessage();
                opMode.telemetry.addData("ERROR", e.getMessage());
                opMode.telemetry.update();
            } finally {
                isAutoShooting = false;
            }
        }).start();
    }

    /**
     * Get distance to target AprilTag using Limelight's ty value and trigonometry
     * This matches the official Limelight distance calculation method
     * @param aprilTagId The target AprilTag ID
     * @return Distance in inches, or -1 if target not found
     */
    private double getDistanceToTarget(int aprilTagId) {
        if (limelight == null) {
            return -1;
        }

        try {
            // Try multiple times to ensure we get a stable reading
            double sumTy = 0;
            int validReadings = 0;

            for (int attempt = 0; attempt < 10; attempt++) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == aprilTagId) {
                            // Get vertical angle to target (ty) - this is the key value!
                            double ty = fiducial.getTargetYDegrees();
                            sumTy += ty;
                            validReadings++;
                            break;
                        }
                    }
                }

                Thread.sleep(50);
            }

            // If we got valid readings, calculate distance
            if (validReadings > 0) {
                double avgTy = sumTy / validReadings;
                return calculateDistanceFromLimelight(avgTy);
            }

        } catch (Exception e) {
            opMode.telemetry.addData("Distance Error", e.getMessage());
        }

        return -1; // Target not found
    }

    /**
     * Calculate distance using official Limelight trigonometry formula
     * Based on: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     *
     * Formula: distance = (goalHeight - lensHeight) / tan(mountAngle + ty)
     *
     * @param targetOffsetAngle_Vertical The ty value from Limelight (degrees)
     * @return Distance from Limelight to goal in inches (pure calculation, no corrections)
     */
    private double calculateDistanceFromLimelight(double targetOffsetAngle_Vertical) {
        double limelightMountAngleDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES;
        double limelightLensHeightInches = LIMELIGHT_LENS_HEIGHT_INCHES;
        double goalHeightInches = GOAL_HEIGHT_INCHES;

        // Calculate angle to goal
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        // Calculate distance using official Limelight formula (pure, no corrections)
        double distanceFromLimelightToGoalInches =
                (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return Math.abs(distanceFromLimelightToGoalInches);
    }

    /**
     * Calculate required shooter RPM based on distance to target
     * Simple threshold-based approach: distance < 100" = 2400 RPM, distance >= 100" = 3000 RPM
     *
     * @param distanceInches Distance to target in inches
     * @return Required RPM
     */
    private double calculateRPMFromDistance(double distanceInches) {
        // Clamp distance to valid range
        distanceInches = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distanceInches));

        // Simple threshold-based RPM selection
        if (distanceInches < DISTANCE_THRESHOLD) {
            return RPM_CLOSE;
        } else {
            return RPM_FAR;
        }
    }

    // Status getters
    public boolean isAutoShooting() {
        return isAutoShooting;
    }

    public int getShotsCompleted() {
        return shotsCompleted;
    }

    public double getLastShotTime() {
        return lastShotTime;
    }

    public double getLastTargetDistance() {
        return lastTargetDistance;
    }

    public double getLastCalculatedRPM() {
        return lastCalculatedRPM;
    }

    public String getCurrentStatus() {
        return currentStatus;
    }

    public void resetShotCounter() {
        shotsCompleted = 0;
        lastShotTime = 0;
        lastTargetDistance = 0;
        lastCalculatedRPM = 0;
    }

    /**
     * Add telemetry information
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("━━━ AUTO-SHOOT STATUS ━━━");
        telemetry.addData("Status", currentStatus);
        telemetry.addData("Active", isAutoShooting ? "YES" : "NO");
        telemetry.addData("Shots Completed", shotsCompleted);

        if (lastTargetDistance > 0) {
            telemetry.addLine();
            telemetry.addLine("━━━ LAST SHOT DATA ━━━");
            telemetry.addData("Distance", "%.1f inches", lastTargetDistance);
            telemetry.addData("RPM Used", "%.0f", lastCalculatedRPM);
            telemetry.addData("Time", "%.1f s ago", sessionTimer.seconds() - lastShotTime);
        }

        telemetry.addLine();
        telemetry.addLine("━━━ CALIBRATION ━━━");
        telemetry.addData("Lens Height", "%.1f in", LIMELIGHT_LENS_HEIGHT_INCHES);
        telemetry.addData("Goal Height", "%.1f in", GOAL_HEIGHT_INCHES);
        telemetry.addData("Mount Angle", "%.2f°", LIMELIGHT_MOUNT_ANGLE_DEGREES);
        telemetry.addData("Distance Threshold", "%.0f in", DISTANCE_THRESHOLD);
        telemetry.addData("AprilTag ID", APRILTAG_ID);
    }
}