package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.IntakeController;
import org.firstinspires.ftc.teamcode.champion.controller.TransferController;
import org.firstinspires.ftc.teamcode.champion.controller.ShooterController;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.LimelightAlignmentController;

/**
 * Streamlined Composite Shooter TeleOp
 * ASSUMPTIONS:
 * - AprilTag is already in vision (no search needed)
 * - 2 balls are already in
 * SEQUENCE:
 * 1. Spin up shooter to target RPM
 * 2. Align to target (within error threshold OR timeout)
 * 3. Stop alignment, wait briefly for stability
 * 4. Shoot immediately
 * KEY FEATURES:
 * - Uses practical alignment threshold (adjustable, default 1.5°)
 * - 2-second alignment timeout ensures shooting happens
 * - Thread-based execution like the working RPM test
 * - Direct error checking without complex state tracking
 * CONTROLS:
 * - Gamepad1 Left Stick Y: Drive forward/backward
 * - Gamepad1 Right Stick X: Turn
 * - Gamepad1 A: Execute auto shoot sequence
 * - Gamepad1 B: Manual alignment toggle
 * - Gamepad1 X: Emergency stop all systems
 * - Gamepad1 Y: Manual shoot (transfer/intake only)
 * - Gamepad1 Right Bumper: Manual shooter toggle
 */
@Config
@TeleOp(name = "Streamlined Composite Shooter", group = "Competition")
public class FirstCompositeTeleop extends LinearOpMode {

    // Controllers
    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;

    // Dashboard config values
    public static double TARGET_RPM = 2850;         // Target RPM for shooting
    public static int APRILTAG_ID = 20;             // AprilTag to align to
    public static double ALIGNMENT_THRESHOLD = 1.5; // Alignment error threshold in degrees
    public static long ALIGNMENT_TIMEOUT = 2000;    // Max time to wait for alignment (ms)
    public static long RPM_TIMEOUT = 3000;          // Max time to wait for RPM (ms)
    public static long SHOOT_DURATION = 1000;       // Duration to run transfer/intake (ms)
    public static long STABILITY_DELAY = 200;       // Delay after stopping alignment (ms)

    // State tracking
    private boolean isAutoShooting = false;
    private boolean isManualAligning = false;
    private boolean shooterOn = false;

    // Button tracking
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastRB = false;

    // Performance tracking
    private int shotsCompleted = 0;
    private double lastShotTime = 0;
    private final ElapsedTime sessionTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize controllers
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);

        try {
            limelightController = new LimelightAlignmentController(this, driveController);
            limelightController.setTargetTag(APRILTAG_ID);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
            telemetry.update();
        }

        // Setup dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== STREAMLINED SHOOTER READY ===");
        telemetry.addLine("Assumptions: AprilTag visible, balls loaded");
        telemetry.addLine("Press A to execute auto shoot sequence");
        telemetry.update();

        waitForStart();
        sessionTimer.reset();

        while (opModeIsActive()) {
            // CRITICAL: Always update shooter PID
            shooterController.updatePID();

            // Manual drive control (disabled during auto shoot)
            if (!isAutoShooting && !isManualAligning) {
                double drive = -gamepad1.left_stick_y * 0.8;
                double turn = gamepad1.right_stick_x * 0.7;
                driveController.arcadeDrive(drive, turn);
            }

            // Update manual alignment if active
            if (isManualAligning) {
                limelightController.align(APRILTAG_ID);
                if (limelightController.hasTarget() &&
                        limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) {
                    telemetry.addLine(">>> ALIGNED - Ready to shoot!");
                }
            }

            // A Button - Auto shoot sequence (thread-based like working version)
            if (gamepad1.a && !lastA && !isAutoShooting) {
                executeAutoShootSequence();
            }
            lastA = gamepad1.a;

            // B Button - Manual alignment toggle
            if (gamepad1.b && !lastB) {
                if (!isManualAligning && !isAutoShooting) {
                    isManualAligning = true;
                    limelightController.startAlignment();
                } else if (isManualAligning) {
                    isManualAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            }
            lastB = gamepad1.b;

            // X Button - Emergency stop
            if (gamepad1.x && !lastX) {
                emergencyStop();
            }
            lastX = gamepad1.x;

            // Y Button - Manual shoot (transfer/intake only)
            if (gamepad1.y && !lastY) {
                manualShoot();
            }
            lastY = gamepad1.y;

            // Right Bumper - Manual shooter toggle
            if (gamepad1.right_bumper && !lastRB) {
                if (!shooterOn) {
                    shooterController.setShooterRPM(TARGET_RPM);
                    shooterOn = true;
                } else {
                    shooterController.shooterStop();
                    shooterOn = false;
                }
            }
            lastRB = gamepad1.right_bumper;

            // Display telemetry
            displayTelemetry();
            telemetry.update();
        }
    }

    private void executeAutoShootSequence() {
        isAutoShooting = true;

        // Use a thread like the working version
        new Thread(() -> {
            try {
                // Step 1: Stop any existing movement
                driveController.stopDrive();

                // Step 2: Start spinning up shooter
                shooterController.setShooterRPM(TARGET_RPM);

                // Step 3: Wait for shooter to reach target RPM (with timeout)
                long rpmStartTime = System.currentTimeMillis();
                while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
                        (System.currentTimeMillis() - rpmStartTime) < RPM_TIMEOUT) {
                    sleep(10);
                }

                // Step 4: Align to target (with error threshold check)
                if (limelightController != null) {
                    limelightController.startAlignment();

                    long alignStartTime = System.currentTimeMillis();

                    // Keep aligning until within threshold OR timeout
                    while (opModeIsActive() &&
                            (System.currentTimeMillis() - alignStartTime) < ALIGNMENT_TIMEOUT) {

                        limelightController.align(APRILTAG_ID);

                        // Check if we're within acceptable error threshold
                        if (limelightController.hasTarget() &&
                                limelightController.getTargetError() <= ALIGNMENT_THRESHOLD) {
                            break;
                        }

                        sleep(20);
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

                    telemetry.addLine(">>> SHOT COMPLETED <<<");
                    telemetry.addData("Shot #", shotsCompleted);
                }

                // Keep shooter running for next shot

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                isAutoShooting = false;
            }
        }).start();
    }

    private void manualShoot() {
        // Manual shoot - just run transfer/intake
        new Thread(() -> {
            try {
                transferController.transferFull();
                intakeController.intakeFull();
                Thread.sleep(SHOOT_DURATION);
                transferController.transferStop();
                intakeController.intakeStop();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    private void emergencyStop() {
        // Stop everything
        isAutoShooting = false;
        isManualAligning = false;
        shooterOn = false;

        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();

        if (limelightController != null) {
            limelightController.stopAlignment();
        }

        telemetry.addLine(">>> EMERGENCY STOP ACTIVATED <<<");
    }

    private void displayTelemetry() {
        telemetry.addLine("╔════════════════════════════╗");
        telemetry.addLine("║  STREAMLINED SHOOTER MODE  ║");
        telemetry.addLine("╚════════════════════════════╝");

        // Status
        telemetry.addLine();
        if (isAutoShooting) {
            telemetry.addLine(">>> AUTO SHOOT IN PROGRESS <<<");
        } else if (isManualAligning) {
            telemetry.addLine(">>> MANUAL ALIGNMENT ACTIVE <<<");
        }

        // Shooter status
        telemetry.addLine();
        telemetry.addLine("── SHOOTER STATUS ──");
        telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
        telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
        telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
        telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✓ YES" : "NO");
        telemetry.addData("Power", "%.3f", shooterController.getShooterPower());

        // Alignment status (when relevant)
        if (limelightController != null && (isManualAligning || isAutoShooting)) {
            telemetry.addLine();
            telemetry.addLine("── ALIGNMENT ──");
            telemetry.addData("Has Target", limelightController.hasTarget() ? "✓" : "✗");
            telemetry.addData("Error", "%.2f°", limelightController.getTargetError());
            telemetry.addData("Threshold", "%.1f°", ALIGNMENT_THRESHOLD);
            telemetry.addData("Within Threshold",
                    limelightController.hasTarget() &&
                            limelightController.getTargetError() <= ALIGNMENT_THRESHOLD ? "✓ YES" : "NO");
        }

        // Session statistics
        telemetry.addLine();
        telemetry.addLine("── SESSION STATS ──");
        telemetry.addData("Shots Completed", shotsCompleted);
        if (shotsCompleted > 0) {
            telemetry.addData("Last Shot", "%.1fs ago",
                    sessionTimer.seconds() - lastShotTime);
            telemetry.addData("Avg Time/Shot", "%.1fs",
                    lastShotTime / shotsCompleted);
        }

        // Controls
        telemetry.addLine();
        telemetry.addLine("── CONTROLS ──");
        telemetry.addLine("A: Auto Shoot | B: Manual Align");
        telemetry.addLine("Y: Manual Feed | RB: Toggle Shooter");
        telemetry.addLine("X: Emergency Stop");

        // Settings
        telemetry.addLine();
        telemetry.addLine("── SETTINGS (Dashboard) ──");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Alignment Threshold", "%.1f°", ALIGNMENT_THRESHOLD);
        telemetry.addData("Alignment Timeout", "%dms", ALIGNMENT_TIMEOUT);
        telemetry.addData("AprilTag ID", APRILTAG_ID);
    }
}