package org.firstinspires.ftc.teamcode.champion.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AutoShootController {
    // Configurable parameters
    public static double TARGET_RPM = 3350;         // Target RPM for shooting
    public static int APRILTAG_ID = 20;             // AprilTag to align to
    public static double ALIGNMENT_THRESHOLD = 5; // Alignment error threshold in degrees
    public static long ALIGNMENT_TIMEOUT = 2000;    // Max time to wait for alignment (ms)
    public static long RPM_TIMEOUT = 3000;          // Max time to wait for RPM (ms)
    public static long SHOOT_DURATION = 1000;       // Duration to run transfer/intake (ms)
    public static long STABILITY_DELAY = 200;       // Delay after stopping alignment (ms)

    private final LinearOpMode opMode;
    private final SixWheelDriveController driveController;
    private final ShooterController shooterController;
    private final IntakeController intakeController;
    private final TransferController transferController;
    private final LimelightAlignmentController limelightController;

    private boolean isAutoShooting = false;
    private int shotsCompleted = 0;
    private double lastShotTime = 0;
    private final ElapsedTime sessionTimer = new ElapsedTime();

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
    }

    public void executeAutoShootSequence(double targetRPM) {
        if (isAutoShooting) return; // Prevent multiple instances

        isAutoShooting = true;

        // Use a thread like the working version
        new Thread(() -> {
            try {
                // Step 1: Stop any existing movement
                driveController.stopDrive();

                // Step 2: Start spinning up shooter
                shooterController.setShooterRPM(targetRPM);

                // Step 3: Wait for shooter to reach target RPM (with timeout)
                long rpmStartTime = System.currentTimeMillis();
                while (opMode.opModeIsActive() && !shooterController.isAtTargetRPM() &&
                        (System.currentTimeMillis() - rpmStartTime) < RPM_TIMEOUT) {
                    Thread.sleep(10);
                }

                // Step 4: Align to target (with error threshold check)
                if (limelightController != null) {
                    limelightController.startAlignment();

                    long alignStartTime = System.currentTimeMillis();
                    boolean alignmentGoodEnough = false;

                    // Keep aligning until within threshold OR timeout
                    while (opMode.opModeIsActive() &&
                            (System.currentTimeMillis() - alignStartTime) < ALIGNMENT_TIMEOUT) {

                        limelightController.align(APRILTAG_ID);

                        // Check if we're within acceptable error threshold
                        if (limelightController.hasTarget() &&
                                limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) {
                            alignmentGoodEnough = true;
                            break;
                        }

                        Thread.sleep(20);
                    }

                    // Stop alignment and motors
                    limelightController.stopAlignment();
                    driveController.stopDrive();

                    // Brief stabilization delay
                    Thread.sleep(STABILITY_DELAY);
                }

                // Step 5: Execute the shot!
                // Ensure shooter is still at RPM
                if (shooterController.isAtTargetRPM() ||
                        Math.abs(shooterController.getRPMError()) < 200) {

                    // Run transfer and intake to shoot
                    transferController.transferFull();
                    intakeController.intakeFull();
                    Thread.sleep(SHOOT_DURATION);

                    // Stop transfer and intake
                    transferController.transferStop();
                    intakeController.intakeStop();

                    shotsCompleted++;
                    lastShotTime = sessionTimer.seconds();

                    opMode.telemetry.addLine(">>> SHOT COMPLETED <<<");
                    opMode.telemetry.addData("Shot #", shotsCompleted);
                }

                // Keep shooter running for next shot

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                isAutoShooting = false;
            }
        }).start();
    }

    public boolean isAutoShooting() {
        return isAutoShooting;
    }

    public int getShotsCompleted() {
        return shotsCompleted;
    }

    public double getLastShotTime() {
        return lastShotTime;
    }

    public void resetShotCounter() {
        shotsCompleted = 0;
        lastShotTime = 0;
    }

    // Add telemetry data to the telemetry object
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("AutoShoot Status", isAutoShooting ? "ACTIVE" : "IDLE");
        telemetry.addData("Shots Completed", shotsCompleted);
        telemetry.addData("Last Shot Time", "%.1f s", lastShotTime);
        telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
        telemetry.addData("AprilTag ID", APRILTAG_ID);
        telemetry.addData("Alignment Threshold", "%.1f°", ALIGNMENT_THRESHOLD);
    }
}