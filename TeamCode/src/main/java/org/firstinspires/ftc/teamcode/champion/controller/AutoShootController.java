package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Locale;

@Config
public class AutoShootController {

    // ========== LIMELIGHT DISTANCE CALIBRATION ==========
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 14.8;
    public static double GOAL_HEIGHT_INCHES = 29.5;
    public static double LIMELIGHT_MOUNT_ANGLE_DEGREES = 19.7;

    // ========== DISTANCE-TO-RPM MAPPING ==========
    public static double DISTANCE_THRESHOLD = 100;
    public static double RPM_CLOSE = 2600;
    public static double RPM_FAR = 3000;
    public static double MIN_DISTANCE = 12;
    public static double MAX_DISTANCE = 200;

    // ========== SHOOTING SEQUENCE PARAMETERS ==========
    public static int APRILTAG_ID = 20;
    public static double ALIGNMENT_THRESHOLD = 1.0;      // Tighter threshold for velocity control
    public static long ALIGNMENT_TIMEOUT = 4000;         // Increased for velocity-based alignment
    public static long RPM_TIMEOUT = 3000;
    public static long SHOOT_DURATION = 1000;
    public static long STABILITY_DELAY = 200;            // Reduced - velocity control is more stable
    public static double RPM_TOLERANCE = 100;
    public static int STABILITY_CHECK_DURATION = 200;    // Time to verify robot is stable (ms)

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

    private String currentStatus = "IDLE";

    public AutoShootController(LinearOpMode opMode,
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

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            this.limelight = null;
            if (opMode.telemetry != null) {
                opMode.telemetry.addData("Warning", "Limelight not found");
            }
        }
    }

    /**
     * Execute complete auto-shoot sequence with velocity-based alignment
     */
    public void executeDistanceBasedAutoShoot() {
        opMode.telemetry.addLine("🔍 DEBUG: executeDistanceBasedAutoShoot() called");
        opMode.telemetry.update();

        if (isAutoShooting) {
            opMode.telemetry.addLine("⚠️ Auto-shoot already in progress!");
            opMode.telemetry.update();
            return;
        }

        isAutoShooting = true;
        opMode.telemetry.addLine("🔍 DEBUG: Starting auto-shoot thread");
        opMode.telemetry.update();

        new Thread(() -> {
            try {
                opMode.telemetry.addLine("🔍 DEBUG: Inside auto-shoot thread");
                opMode.telemetry.update();

                // STEP 1: Find distance to target
                currentStatus = "FINDING DISTANCE";
                opMode.telemetry.addLine("🔍 DEBUG: Finding distance to target");
                opMode.telemetry.update();
                double distance = getDistanceToTarget(APRILTAG_ID);

                if (distance < 0) {
                    currentStatus = "TARGET NOT FOUND";
                    opMode.telemetry.addLine("❌ Cannot find target AprilTag!");
                    opMode.telemetry.update();
                    sleep(2000);
                    return;
                }

                lastTargetDistance = distance;
                opMode.telemetry.addData("🔍 DEBUG: Target distance found", "%.1f inches", distance);
                opMode.telemetry.update();

                // STEP 2: Calculate required RPM
                currentStatus = "CALCULATING RPM";
                opMode.telemetry.addLine("🔍 DEBUG: Calculating required RPM");
                opMode.telemetry.update();
                double targetRPM = calculateRPMFromDistance(distance);
                lastCalculatedRPM = targetRPM;

                opMode.telemetry.addLine("=== AUTO-SHOOT SEQUENCE ===");
                opMode.telemetry.addData("Distance", String.format(Locale.US, "%.1f inches", distance));
                opMode.telemetry.addData("Target RPM", String.format(Locale.US, "%.0f", targetRPM));
                opMode.telemetry.update();
                sleep(500);

                // STEP 3: Start shooter (do this BEFORE alignment to save time)
                currentStatus = "SPINNING UP";
                opMode.telemetry.addLine("🔍 DEBUG: Starting shooter spin-up");
                opMode.telemetry.update();
                shooterController.setShooterRPM(targetRPM);

                // STEP 4: Align to target using VELOCITY-BASED CONTROL
                currentStatus = "ALIGNING";
                opMode.telemetry.addLine("🔍 DEBUG: Starting alignment phase");
                opMode.telemetry.update();
                if (limelightController != null) {
                    opMode.telemetry.addLine("🔍 DEBUG: Limelight controller available");
                    opMode.telemetry.update();

                    // Set drive controller to velocity mode
                    opMode.telemetry.addLine("🔍 DEBUG: Setting drive controller to velocity mode");
                    opMode.telemetry.update();
                    driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

                    opMode.telemetry.addLine("🔍 DEBUG: Starting limelight alignment");
                    opMode.telemetry.update();
                    limelightController.startAlignment();

                    ElapsedTime alignTimer = new ElapsedTime();
                    boolean alignmentAchieved = false;

                    opMode.telemetry.addLine("🔍 DEBUG: Entering alignment loop");
                    opMode.telemetry.update();

                    // Alignment loop - let the controller handle it
                    while (opMode.opModeIsActive() && alignTimer.milliseconds() < ALIGNMENT_TIMEOUT) {

                        // Update alignment (this uses velocity control internally)
                        opMode.telemetry.addData("🔍 DEBUG: Calling limelightController.align()", APRILTAG_ID);
                        limelightController.align(APRILTAG_ID);

                        double alignmentError = limelightController.getTargetError();

                        opMode.telemetry.addLine("🎯 Aligning to target...");
                        opMode.telemetry.addData("Error", String.format(Locale.US, "%.2f°", alignmentError));
                        opMode.telemetry.addData("Threshold", String.format(Locale.US, "%.2f°", ALIGNMENT_THRESHOLD));
                        opMode.telemetry.addData("State", limelightController.getState());
                        opMode.telemetry.addData("Zone", limelightController.getCurrentZone());
                        opMode.telemetry.addData("Has Target", limelightController.hasTarget() ? "YES" : "NO");
                        opMode.telemetry.addData("Time in Alignment", "%.1f s", alignTimer.seconds());
                        opMode.telemetry.update();

                        // Check if aligned using the controller's built-in state
                        if (limelightController.isAligned()) {
                            alignmentAchieved = true;
                            opMode.telemetry.addLine("🔍 DEBUG: Alignment achieved!");
                            opMode.telemetry.update();
                            break;
                        }

                        sleep(20);
                    }

                    opMode.telemetry.addLine("🔍 DEBUG: Exiting alignment loop");
                    opMode.telemetry.update();

                    // Stop alignment
                    opMode.telemetry.addLine("🔍 DEBUG: Stopping limelight alignment");
                    opMode.telemetry.update();
                    limelightController.stopAlignment();
                    driveController.stopDrive();

                    if (!alignmentAchieved) {
                        opMode.telemetry.addLine("⚠️ Alignment timeout - shooting anyway");
                        opMode.telemetry.addData("🔍 DEBUG: Alignment failed after", "%.1f s", alignTimer.seconds());
                        opMode.telemetry.update();
                    } else {
                        opMode.telemetry.addLine("🔍 DEBUG: Alignment successful, verifying stability");
                        opMode.telemetry.update();

                        // Verify stability before shooting
                        if (!verifyStability()) {
                            opMode.telemetry.addLine("⚠️ Robot not stable - retrying alignment");
                            opMode.telemetry.update();
                            sleep(1000);
                            return;
                        }

                        opMode.telemetry.addLine("🔍 DEBUG: Stability verified");
                        opMode.telemetry.update();
                    }

                    // Brief stabilization delay
                    opMode.telemetry.addLine("🔍 DEBUG: Stabilization delay");
                    opMode.telemetry.update();
                    sleep(STABILITY_DELAY);
                }

                // STEP 5: Wait for shooter to reach RPM (if not already there)
                currentStatus = "VERIFYING RPM";
                opMode.telemetry.addLine("🔍 DEBUG: Verifying shooter RPM");
                opMode.telemetry.update();
                ElapsedTime rpmTimer = new ElapsedTime();
                while (opMode.opModeIsActive() &&
                        !shooterController.isAtTargetRPM() &&
                        rpmTimer.milliseconds() < RPM_TIMEOUT) {

                    opMode.telemetry.addLine("⚙️ Waiting for shooter RPM...");
                    opMode.telemetry.addData("Current", String.format(Locale.US, "%.0f RPM", shooterController.getShooterRPM()));
                    opMode.telemetry.addData("Target", String.format(Locale.US, "%.0f RPM", targetRPM));
                    opMode.telemetry.addData("RPM Error", String.format(Locale.US, "%.0f", shooterController.getRPMError()));
                    opMode.telemetry.addData("Time Waiting", "%.1f s", rpmTimer.seconds());
                    opMode.telemetry.update();

                    sleep(50);
                }

                // STEP 6: Execute shot
                currentStatus = "SHOOTING";
                opMode.telemetry.addLine("🔍 DEBUG: Preparing to execute shot");
                opMode.telemetry.update();

                if (shooterController.isAtTargetRPM() ||
                        Math.abs(shooterController.getRPMError()) < RPM_TOLERANCE) {

                    opMode.telemetry.addLine("🔥 FIRING!");
                    opMode.telemetry.update();

                    opMode.telemetry.addLine("🔍 DEBUG: Starting transfer and intake");
                    opMode.telemetry.update();
                    transferController.transferFull();
                    intakeController.intakeFull();
                    sleep(SHOOT_DURATION);

                    opMode.telemetry.addLine("🔍 DEBUG: Stopping transfer and intake");
                    opMode.telemetry.update();
                    transferController.transferStop();
                    intakeController.intakeStop();

                    shotsCompleted++;
                    lastShotTime = sessionTimer.seconds();

                    currentStatus = "SHOT COMPLETE";

                    opMode.telemetry.clear();
                    opMode.telemetry.addLine("=== ✅ SHOT COMPLETE ===");
                    opMode.telemetry.addData("Shot #", shotsCompleted);
                    opMode.telemetry.addData("Distance", String.format(Locale.US, "%.1f in", lastTargetDistance));
                    opMode.telemetry.addData("RPM Used", String.format(Locale.US, "%.0f", lastCalculatedRPM));
                    if (limelightController != null) {
                        opMode.telemetry.addData("Align Time", String.format(Locale.US, "%.2fs",
                                limelightController.getTotalAlignmentTime()));
                    }
                    opMode.telemetry.update();
                    opMode.telemetry.addLine("🔍 DEBUG: Auto-shoot sequence completed successfully");
                    opMode.telemetry.update();

                } else {
                    currentStatus = "RPM NOT READY";
                    opMode.telemetry.addLine("❌ Shooter not at RPM - shot aborted");
                    opMode.telemetry.addData("🔍 DEBUG: Current RPM", "%.0f", shooterController.getShooterRPM());
                    opMode.telemetry.addData("🔍 DEBUG: Target RPM", "%.0f", targetRPM);
                    opMode.telemetry.addData("🔍 DEBUG: RPM Error", "%.0f", shooterController.getRPMError());
                    opMode.telemetry.addData("🔍 DEBUG: Tolerance", "%.0f", RPM_TOLERANCE);
                    opMode.telemetry.update();
                }

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                currentStatus = "INTERRUPTED";
                opMode.telemetry.addLine("🔍 DEBUG: Auto-shoot thread interrupted");
                opMode.telemetry.addData("ERROR", e.getMessage());
                opMode.telemetry.update();
            } catch (Exception e) {
                currentStatus = "ERROR: " + e.getMessage();
                opMode.telemetry.addData("ERROR", e.getMessage());
                opMode.telemetry.addLine("🔍 DEBUG: Auto-shoot thread caught exception");
                opMode.telemetry.update();
            } finally {
                isAutoShooting = false;
                opMode.telemetry.addLine("🔍 DEBUG: Auto-shoot thread finished, isAutoShooting set to false");
                opMode.telemetry.update();
            }
        }).start();
    }

    /**
     * Verify robot is stable before shooting
     * Checks that wheel velocities are near zero
     */
    private boolean verifyStability() throws InterruptedException {
        opMode.telemetry.addLine("🔍 DEBUG: Verifying stability");
        opMode.telemetry.update();

        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() < STABILITY_CHECK_DURATION) {
            double leftVel = Math.abs(driveController.getLeftVelocity());
            double rightVel = Math.abs(driveController.getRightVelocity());

            opMode.telemetry.addData("🔍 DEBUG: Left Velocity", "%.0f ticks/sec", leftVel);
            opMode.telemetry.addData("🔍 DEBUG: Right Velocity", "%.0f ticks/sec", rightVel);
            opMode.telemetry.addData("🔍 DEBUG: Stability Check Time", "%.1f ms", timer.milliseconds());
            opMode.telemetry.update();

            // If either wheel is moving too fast, not stable
            if (leftVel > 50 || rightVel > 50) {  // 50 ticks/sec threshold
                opMode.telemetry.addLine("🔍 DEBUG: Robot not stable - velocity too high");
                opMode.telemetry.update();
                return false;
            }

            sleep(20);
        }

        opMode.telemetry.addLine("🔍 DEBUG: Robot is stable");
        opMode.telemetry.update();
        return true;
    }

    /**
     * Get distance to target AprilTag using Limelight trigonometry
     */
    private double getDistanceToTarget(int aprilTagId) {
        if (limelight == null) {
            return -1;
        }

        try {
            double sumTy = 0;
            int validReadings = 0;

            for (int attempt = 0; attempt < 10; attempt++) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == aprilTagId) {
                            double ty = fiducial.getTargetYDegrees();
                            sumTy += ty;
                            validReadings++;
                            break;
                        }
                    }
                }

                sleep(50);
            }

            if (validReadings > 0) {
                double avgTy = sumTy / validReadings;
                return calculateDistanceFromLimelight(avgTy);
            }

        } catch (Exception e) {
            opMode.telemetry.addData("Distance Error", e.getMessage());
        }

        return -1;
    }

    /**
     * Calculate distance using Limelight trigonometry
     * Formula: distance = (goalHeight - lensHeight) / tan(mountAngle + ty)
     */
    private double calculateDistanceFromLimelight(double targetOffsetAngle_Vertical) {
        double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        double distanceFromLimelightToGoalInches =
                (GOAL_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

        return Math.abs(distanceFromLimelightToGoalInches);
    }

    /**
     * Calculate required shooter RPM based on distance
     */
    private double calculateRPMFromDistance(double distanceInches) {
        distanceInches = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distanceInches));

        if (distanceInches < DISTANCE_THRESHOLD) {
            return RPM_CLOSE;
        } else {
            return RPM_FAR;
        }
    }

    /**
     * Helper method to replace Thread.sleep with proper exception handling
     */
    private void sleep(long milliseconds) throws InterruptedException {
        Thread.sleep(milliseconds);
    }

    // ========== GETTERS ==========

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
        telemetry.addLine("┌─── AUTO-SHOOT STATUS ───┐");
        telemetry.addData("│ Status", currentStatus);
        telemetry.addData("│ Active", isAutoShooting ? "YES" : "NO");
        telemetry.addData("│ Shots Completed", shotsCompleted);

        if (lastTargetDistance > 0) {
            telemetry.addLine("├─── LAST SHOT DATA ───");
            telemetry.addData("│ Distance", String.format(Locale.US, "%.1f inches", lastTargetDistance));
            telemetry.addData("│ RPM Used", String.format(Locale.US, "%.0f", lastCalculatedRPM));
            telemetry.addData("│ Time", String.format(Locale.US, "%.1f s ago", sessionTimer.seconds() - lastShotTime));
        }

        telemetry.addLine("├─── CALIBRATION ───");
        telemetry.addData("│ Lens Height", String.format(Locale.US, "%.1f in", LIMELIGHT_LENS_HEIGHT_INCHES));
        telemetry.addData("│ Goal Height", String.format(Locale.US, "%.1f in", GOAL_HEIGHT_INCHES));
        telemetry.addData("│ Mount Angle", String.format(Locale.US, "%.2f°", LIMELIGHT_MOUNT_ANGLE_DEGREES));
        telemetry.addData("│ AprilTag ID", APRILTAG_ID);
        telemetry.addLine("└───────────────────────┘");
    }
}