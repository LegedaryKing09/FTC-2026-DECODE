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
 * Simplified RPM Test TeleOp
 *
 * CONTROLS:
 * - Gamepad1 Left Stick Y: Drive forward/backward
 * - Gamepad1 Right Stick X: Turn
 * - Gamepad1 A: Align to AprilTag
 * - Gamepad1 B: Auto intake and shoot at target RPM
 * - Gamepad1 X: Emergency stop all systems
 * - Gamepad1 Start: Save current RPM data to log
 * - Gamepad1 Back: Clear saved data log
 *
 * Use FTC Dashboard to change TEST_RPM value in real-time
 */
@Config
@TeleOp(name = "Simplified Shooter RPM Test", group = "Test")
public class ShooterRPMTestTeleop extends LinearOpMode {

    // Controllers
    SixWheelDriveController driveController;
    TransferController transferController;
    ShooterController shooterController;
    IntakeController intakeController;
    LimelightAlignmentController limelightController;

    // Dashboard config values - change these in real-time via FTC Dashboard
    public static double TEST_RPM = 2850;  // Target RPM - adjustable via dashboard
    public static int APRILTAG_ID = 20;    // AprilTag to align to
    public static long ALIGNMENT_TIMEOUT = 3000;  // Alignment timeout in ms
    public static long SHOOT_DURATION = 1500;     // How long to run transfer/intake during shoot

    // Data collection
    private StringBuilder dataLog = new StringBuilder();
    private int shotCount = 0;
    private ElapsedTime sessionTimer = new ElapsedTime();

    // State tracking
    private boolean isAligning = false;
    private boolean isShooting = false;

    // Button press tracking
    boolean isPressingA = false;
    boolean isPressingB = false;
    boolean isPressingX = false;
    boolean isPressingStart = false;
    boolean isPressingBack = false;

    @Override
    public void runOpMode() {
        // Initialize controllers
        driveController = new SixWheelDriveController(this);
        transferController = new TransferController(this);
        shooterController = new ShooterController(this);
        intakeController = new IntakeController(this);

        try {
            limelightController = new LimelightAlignmentController(this);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to init Limelight: " + e.getMessage());
            telemetry.update();
        }

        // Setup dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize data log header
        dataLog.append("=== SIMPLIFIED SHOOTER RPM TEST SESSION ===\n");
        dataLog.append("Time(s), Shot#, Target RPM, Actual RPM, RPM Error, Success\n");

        sessionTimer.reset();

        telemetry.addLine("=== SIMPLIFIED SHOOTER RPM TEST MODE ===");
        telemetry.addLine("Change TEST_RPM via FTC Dashboard");
        telemetry.addLine("Press B to auto intake and shoot");
        telemetry.update();

        waitForStart();
        sessionTimer.reset();

        while (opModeIsActive()) {
            // CRITICAL: Always update PID
            shooterController.updatePID();

            // Drive control (reduced speed for testing precision)
            double drive = -gamepad1.left_stick_y * 0.7;
            double turn = gamepad1.right_stick_x * 0.6;

            if (!isAligning) {
                driveController.arcadeDrive(drive, turn);
            }

            // A Button - Toggle alignment to AprilTag
            if (gamepad1.a && !isPressingA) {
                isPressingA = true;
                if (!isAligning) {
                    isAligning = true;
                    limelightController.startAlignment();
                } else {
                    isAligning = false;
                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }
            } else if (!gamepad1.a && isPressingA) {
                isPressingA = false;
            }

            // Update alignment if active
            if (isAligning) {
                limelightController.align(APRILTAG_ID);
                if (limelightController.isAligned()) {
                    telemetry.addLine(">>> ALIGNED - Ready to shoot!");
                }
            }

            // B Button - Auto intake and shoot sequence
            if (gamepad1.b && !isPressingB && !isShooting) {
                isPressingB = true;
                executeAutoShootSequence();
            } else if (!gamepad1.b && isPressingB) {
                isPressingB = false;
            }

            // X Button - Emergency stop
            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                emergencyStop();
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            // Start button - Save data point
            if (gamepad1.start && !isPressingStart) {
                isPressingStart = true;
                saveDataPoint(true);  // Mark as successful
            } else if (!gamepad1.start && isPressingStart) {
                isPressingStart = false;
            }

            // Back button - Clear log
            if (gamepad1.back && !isPressingBack) {
                isPressingBack = true;
                clearDataLog();
            } else if (!gamepad1.back && isPressingBack) {
                isPressingBack = false;
            }

            // Display telemetry
            displayTestTelemetry();
            telemetry.update();
        }
    }

    private void executeAutoShootSequence() {
        isShooting = true;

        // Start a thread to handle the complete shooting sequence
        new Thread(() -> {
            try {
                // Step 1: Start intake to collect ball
                intakeController.intakeFull();

                // Step 2: Drive forward slightly to intake ball (like in WorkingCompositeTeleop)
                driveController.tankDrive(0.8, 0.8);
                Thread.sleep(300);
                driveController.tankDrive(-0.8, -0.8);
                Thread.sleep(300);
                driveController.stopDrive();

                // Step 3: Stop intake and transfer
                intakeController.intakeStop();
                transferController.transferStop();

                // Step 4: Set target RPM and wait for shooter to reach it
                shooterController.setShooterRPM(TEST_RPM);

                // Step 5: Align if limelight is available (optional - can be skipped if already aligned)
                if (limelightController != null && !isAligning) {
                    limelightController.startAlignment();

                    long alignmentStartTime = System.currentTimeMillis();
                    while (opModeIsActive() && !limelightController.isAligned() &&
                            (System.currentTimeMillis() - alignmentStartTime) < ALIGNMENT_TIMEOUT) {
                        limelightController.align(APRILTAG_ID);
                        Thread.sleep(20);
                    }

                    limelightController.stopAlignment();
                    driveController.stopDrive();
                }

                // Step 6: Wait for shooter to reach target RPM
                long shooterStartTime = System.currentTimeMillis();
                long shooterTimeout = 4000; // 4 second timeout

                while (opModeIsActive() && !shooterController.isAtTargetRPM() &&
                        (System.currentTimeMillis() - shooterStartTime) < shooterTimeout) {
                    Thread.sleep(10);
                }

                // Step 7: If at target RPM, execute the shot
                if (shooterController.isAtTargetRPM()) {
                    // Log the shot data
                    saveDataPoint(false);  // Will be marked successful manually if needed

                    // Wait a moment for stability
                    Thread.sleep(750);

                    // Execute transfer and intake to shoot
                    transferController.transferFull();
                    intakeController.intakeFull();
                    Thread.sleep(SHOOT_DURATION);

                    // Stop transfer and intake
                    transferController.transferStop();
                    intakeController.intakeStop();

                    shotCount++;

                    telemetry.addLine(">>> SHOT COMPLETED <<<");
                    telemetry.addData("Shot #", shotCount);
                    telemetry.update();
                }

                // Keep shooter running for next shot - don't stop automatically

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } finally {
                isShooting = false;
            }
        }).start();
    }

    private void emergencyStop() {
        shooterController.shooterStop();
        intakeController.intakeStop();
        transferController.transferStop();
        driveController.stopDrive();

        if (isAligning) {
            limelightController.stopAlignment();
            isAligning = false;
        }

        isShooting = false;

        telemetry.addLine(">>> EMERGENCY STOP ACTIVATED <<<");
    }

    private void saveDataPoint(boolean wasSuccessful) {
        double currentRPM = shooterController.getShooterRPM();
        double targetRPM = shooterController.getTargetRPM();
        double error = shooterController.getRPMError();

        String dataPoint = String.format("%.1f, %d, %.0f, %.0f, %.0f, %s\n",
                sessionTimer.seconds(),
                shotCount,
                targetRPM,
                currentRPM,
                error,
                wasSuccessful ? "YES" : "NO"
        );

        dataLog.append(dataPoint);

        telemetry.addLine(">>> DATA SAVED <<<");
        telemetry.addData("Entry", dataPoint);
    }

    private void clearDataLog() {
        dataLog = new StringBuilder();
        dataLog.append("=== SIMPLIFIED SHOOTER RPM TEST SESSION ===\n");
        dataLog.append("Time(s), Shot#, Target RPM, Actual RPM, RPM Error, Success\n");
        shotCount = 0;
        sessionTimer.reset();

        telemetry.addLine(">>> DATA LOG CLEARED <<<");
    }

    private void displayTestTelemetry() {
        telemetry.addLine("╔══════════════════════════════╗");
        telemetry.addLine("║   SIMPLIFIED RPM TEST MODE   ║");
        telemetry.addLine("╚══════════════════════════════╝");

        // Current settings
        telemetry.addLine();
        telemetry.addData("Target RPM", "%.0f (Dashboard)", TEST_RPM);

        // Shooter status
        telemetry.addLine();
        telemetry.addLine("── SHOOTER STATUS ──");
        telemetry.addData("Current RPM", "%.0f", shooterController.getShooterRPM());
        telemetry.addData("RPM Error", "%.0f", shooterController.getRPMError());
        telemetry.addData("At Target", shooterController.isAtTargetRPM() ? "✓ YES" : "NO");
        telemetry.addData("Shooter Power", "%.3f", shooterController.getShooterPower());

        // Alignment status
        if (isAligning && limelightController != null) {
            telemetry.addLine();
            telemetry.addLine("── ALIGNMENT ──");
            telemetry.addData("State", limelightController.getState());
            telemetry.addData("Has Target", limelightController.hasTarget());
            telemetry.addData("Error", "%.2f°", limelightController.getTargetError());
            telemetry.addData("Aligned", limelightController.isAligned() ? "✓ YES" : "NO");
        }

        // Session info
        telemetry.addLine();
        telemetry.addLine("── SESSION INFO ──");
        telemetry.addData("Shots Taken", shotCount);
        telemetry.addData("Session Time", "%.1fs", sessionTimer.seconds());

        // Current status
        if (isShooting) {
            telemetry.addLine();
            telemetry.addLine(">>> AUTO SHOOT SEQUENCE IN PROGRESS <<<");
        }

        // Controls reminder
        telemetry.addLine();
        telemetry.addLine("── CONTROLS ──");
        telemetry.addLine("A: Toggle Alignment | B: Auto Shoot");
        telemetry.addLine("X: Emergency Stop");
        telemetry.addLine("Start: Save Data | Back: Clear Log");
        telemetry.addLine("Change TEST_RPM via FTC Dashboard");

        // Data log preview (last 3 entries)
        telemetry.addLine();
        telemetry.addLine("── RECENT DATA ──");
        String[] lines = dataLog.toString().split("\n");
        int startIdx = Math.max(2, lines.length - 3);  // Skip header lines
        for (int i = startIdx; i < lines.length; i++) {
            if (!lines[i].trim().isEmpty()) {
                telemetry.addLine(lines[i]);
            }
        }
    }
}