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
public class NewAutoShootController {

    // ========== CALIBRATION ==========
    public static double LIMELIGHT_LENS_HEIGHT_INCHES = 14.8;
    public static double GOAL_HEIGHT_INCHES = 29.5;
    public static double LIMELIGHT_MOUNT_ANGLE_DEGREES = 21.41;
    public static double DISTANCE_THRESHOLD = 100;
    public static double RPM_CLOSE = 2800;
    public static double RAMP_ANGLE_CLOSE = 121;
    public static double RPM_FAR = 3150;
    public static double RAMP_ANGLE_FAR = 119.5;

    // ========== SEQUENCE PARAMETERS ==========
    public static int APRILTAG_ID = 20;
    public static double ALIGNMENT_THRESHOLD = 1.0;
    public static long ALIGNMENT_TIMEOUT = 1000;
    public static long RPM_TIMEOUT = 1500;
    public static long SHOOT_DURATION = 1500;
    public static double RPM_TOLERANCE = 150;

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final NewShooterController shooterController;
    private final NewIntakeController intakeController;
    private final NewTransferController transferController;
    private final UptakeController uptakeController;
    private final LimelightAlignmentController limelightController;
    private final NewRampController rampController;
    private Limelight3A limelight;

    private volatile boolean isAutoShooting = false;
    private String currentStatus = "IDLE";
    private int shotsCompleted = 0;

    // Single reusable timer
    private final ElapsedTime timer = new ElapsedTime();

    // Tracking variables
    private double lastShotTime = 0;
    private double lastTargetDistance = 0;
    private double lastCalculatedRPM = 0;
    private double lastRampAngle = 0;
    private final ElapsedTime sessionTimer = new ElapsedTime();

    public NewAutoShootController(LinearOpMode opMode,
                               SixWheelDriveController driveController,
                               NewShooterController shooterController,
                               NewIntakeController intakeController,
                               NewTransferController transferController,
                               UptakeController uptakeController,
                               LimelightAlignmentController limelightController,
                               NewRampController rampController) {
        this.opMode = opMode;
        this.driveController = driveController;
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.transferController = transferController;
        this.uptakeController = uptakeController;
        this.limelightController = limelightController;
        this.rampController = rampController;

        try {
            this.limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            this.limelight = null;
        }
    }

    public boolean executeDistanceBasedAutoShoot(boolean adjustRamp, boolean adjustRPM, boolean stopShooterAfter) {
        if (isAutoShooting) {
            return false;
        }

        isAutoShooting = true;
        boolean success = false;

        try {
            // Step 1: Get distance if needed
            double distance = -1;
            if (adjustRamp || adjustRPM) {
                distance = getDistanceToTarget(APRILTAG_ID);
                if (distance < 0) {
                    currentStatus = "NO_TARGET";
                    return false;
                }
                lastTargetDistance = distance;
            }

            // Step 2: Calculate settings
            double targetRPM = adjustRPM ? calculateRPMFromDistance(distance) : shooterController.getTargetRPM();
            double targetRampAngle = adjustRamp ? calculateRampAngleFromDistance(distance) :
                    (rampController != null ? rampController.getTargetAngle() : RAMP_ANGLE_CLOSE);

            // Track calculated values
            lastCalculatedRPM = targetRPM;
            lastRampAngle = targetRampAngle;

            // Step 3: Apply settings
            if (adjustRamp && rampController != null) {
                rampController.setTargetAngle(targetRampAngle);
                sleep(200); // Brief wait for ramp
            }

            if (adjustRPM) {
                shooterController.setTargetRPM(targetRPM);
                shooterController.startShooting();
            }

            // Step 4: Align
            boolean aligned = false;
            if (limelightController != null) {
                currentStatus = "ALIGNING";
                driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);
                limelightController.startAlignment();

                timer.reset();
                int telemetryCounter = 0;

                while (opMode.opModeIsActive() && timer.milliseconds() < ALIGNMENT_TIMEOUT) {
                    limelightController.align(APRILTAG_ID);

                    // Update telemetry only every 250ms
                    if (telemetryCounter++ % 12 == 0) {
                        opMode.telemetry.addData("Aligning", "%.1f°", limelightController.getTargetError());
                        opMode.telemetry.update();
                    }

                    if (limelightController.isAligned()) {
                        aligned = true;
                        break;
                    }

                    sleep(20);
                }

                limelightController.stopAlignment();
                driveController.stopDrive();
            }

            // Step 5: Wait for RPM
            currentStatus = "RPM_WAIT";
            timer.reset();
            while (opMode.opModeIsActive() && !shooterController.isAtTargetRPM() && timer.milliseconds() < RPM_TIMEOUT) {
                shooterController.update();
                sleep(50);
            }

            // Step 6: Shoot with NEW sequence (Intake → Transfer → UPTAKE → Shooter)
            if (shooterController.isAtTargetRPM() || Math.abs(shooterController.getRPMError()) < RPM_TOLERANCE) {
                currentStatus = "SHOOTING";

                // Start all systems for shooting
                transferController.setState(true);
                transferController.update();

                uptakeController.setState(true);
                uptakeController.update();

                intakeController.setState(true);
                intakeController.update();

                sleep(SHOOT_DURATION);

                // Stop all systems
                transferController.setState(false);
                transferController.update();

                uptakeController.setState(false);
                uptakeController.update();

                intakeController.setState(false);
                intakeController.update();

                shotsCompleted++;
                lastShotTime = sessionTimer.seconds();
                success = true;

                opMode.telemetry.addLine("Shot Complete!");
                opMode.telemetry.update();
            }

            // Step 7: Stop shooter if requested
            if (stopShooterAfter) {
                shooterController.stopShooting();
            }

        } catch (Exception e) {
            currentStatus = "ERROR";
        } finally {
            isAutoShooting = false;
        }

        return success;
    }

    /**
     * Default auto-shoot for TeleOp
     */
    public void executeDistanceBasedAutoShoot() {
        executeDistanceBasedAutoShoot(true, true, true);
    }

    /**
     * Distance calculation
     */
    private double getDistanceToTarget(int aprilTagId) {
        if (limelight == null) return -1;

        try {
            double sumTy = 0;
            int validReadings = 0;

            for (int i = 0; i < 3; i++) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == aprilTagId) {
                            sumTy += fiducial.getTargetYDegrees();
                            validReadings++;
                            break;
                        }
                    }
                }
                sleep(30);
            }

            if (validReadings > 0) {
                double avgTy = sumTy / validReadings;
                double angleRad = Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEGREES + avgTy);
                return Math.abs((GOAL_HEIGHT_INCHES - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleRad));
            }
        } catch (Exception e) {
            // Silent fail
        }
        return -1;
    }

    private double calculateRPMFromDistance(double distance) {
        return (distance < DISTANCE_THRESHOLD) ? RPM_CLOSE : RPM_FAR;
    }

    private double calculateRampAngleFromDistance(double distance) {
        return (distance < DISTANCE_THRESHOLD) ? RAMP_ANGLE_CLOSE : RAMP_ANGLE_FAR;
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // ========== GETTERS ==========

    public boolean isNotAutoShooting() {
        return !isAutoShooting;
    }

    public String getCurrentStatus() {
        return currentStatus;
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

    public double getLastRampAngle() {
        return lastRampAngle;
    }

    public void resetShotCounter() {
        shotsCompleted = 0;
        lastShotTime = 0;
        lastTargetDistance = 0;
        lastCalculatedRPM = 0;
        lastRampAngle = 0;
    }

    /**
     * Get the ID of the currently visible AprilTag
     */
    public int getVisibleAprilTagId() {
        if (limelight == null) return -1;

        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    return (int) fiducials.get(0).getFiducialId();
                }
            }
        } catch (Exception e) {
            // Ignore exceptions
        }

        return -1;
    }

    /**
     * Add telemetry information
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("┌─── AUTO-SHOOT STATUS ───");
        telemetry.addData("│ Status", currentStatus);
        telemetry.addData("│ Active", isAutoShooting ? "YES" : "NO");
        telemetry.addData("│ Shots Completed", shotsCompleted);

        if (lastTargetDistance > 0) {
            telemetry.addLine("├─── LAST SHOT DATA ───");
            telemetry.addData("│ Distance", String.format(Locale.US, "%.1f inches", lastTargetDistance));
            telemetry.addData("│ RPM Used", String.format(Locale.US, "%.0f", lastCalculatedRPM));
            telemetry.addData("│ Ramp Angle", String.format(Locale.US, "%.1f°", lastRampAngle));
            telemetry.addData("│ Time", String.format(Locale.US, "%.1f s ago", sessionTimer.seconds() - lastShotTime));
        }

        telemetry.addLine("├─── SHOOTING SETTINGS ───");
        telemetry.addData("│ Close: ", String.format(Locale.US, "%.0f RPM @ %.1f°", RPM_CLOSE, RAMP_ANGLE_CLOSE));
        telemetry.addData("│ Far: ", String.format(Locale.US, "%.0f RPM @ %.1f°", RPM_FAR, RAMP_ANGLE_FAR));
        telemetry.addData("│ Distance Threshold", String.format(Locale.US, "%.0f in", DISTANCE_THRESHOLD));

        telemetry.addLine("├─── CALIBRATION ───");
        telemetry.addData("│ Lens Height", String.format(Locale.US, "%.1f in", LIMELIGHT_LENS_HEIGHT_INCHES));
        telemetry.addData("│ Goal Height", String.format(Locale.US, "%.1f in", GOAL_HEIGHT_INCHES));
        telemetry.addData("│ Mount Angle", String.format(Locale.US, "%.2f°", LIMELIGHT_MOUNT_ANGLE_DEGREES));
        telemetry.addData("│ AprilTag ID", APRILTAG_ID);
        telemetry.addLine("└──────────────────────");
    }
}