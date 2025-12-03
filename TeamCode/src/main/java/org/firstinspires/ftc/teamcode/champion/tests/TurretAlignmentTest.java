package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.TurretAlignmentController;
import org.firstinspires.ftc.teamcode.champion.controller.TurretController;

/**
 * Test OpMode for TurretAlignmentController
 *
 * CONTROLS:
 * - DPAD_UP: Start alignment to target tag
 * - DPAD_DOWN: Stop alignment
 * - DPAD_LEFT: Change target tag ID (decrement)
 * - DPAD_RIGHT: Change target tag ID (increment)
 * - A: Toggle continuous alignment mode
 * - B: Emergency stop
 * - X: Display detailed telemetry
 * - Y: Reset and restart alignment
 */
@TeleOp(name = "Turret Alignment Test", group = "Test")
public class TurretAlignmentTest extends LinearOpMode {

    private TurretController turretController;
    private TurretAlignmentController alignmentController;

    private int targetTagId = 20;
    private boolean continuousMode = false;
    private boolean detailedTelemetry = false;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime testTimer = new ElapsedTime();

    // Test statistics
    private int alignmentAttempts = 0;
    private int successfulAlignments = 0;
    private double totalAlignmentTime = 0;
    private double bestAlignmentTime = Double.MAX_VALUE;
    private double worstAlignmentTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Turret Alignment Test...");
        telemetry.update();

        try {
            // Initialize controllers
            turretController = new TurretController(this);
            alignmentController = new TurretAlignmentController(this, turretController);
            alignmentController.setTargetTag(targetTagId);

            telemetry.addLine("✓ Turret Controller initialized");
            telemetry.addLine("✓ Alignment Controller initialized");
            telemetry.addLine();
            telemetry.addLine("Ready to start!");
            telemetry.addLine();
            displayControls();
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("❌ INITIALIZATION FAILED");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            return;
        }

        waitForStart();
        runtime.reset();
        testTimer.reset();

        while (opModeIsActive()) {
            // Handle button inputs
            handleControls();

            // Update controllers
            turretController.update();
            alignmentController.update();

            // Handle continuous mode
            if (continuousMode && alignmentController.isAligned()) {
                // Wait a bit, then restart alignment
                sleep(500);
                restartAlignment();
            }

            // Update statistics when alignment completes
            updateStatistics();

            // Display telemetry
            if (detailedTelemetry) {
                displayDetailedTelemetry();
            } else {
                displayStandardTelemetry();
            }

            telemetry.update();
        }

        // Cleanup
        alignmentController.stopAlignment();
        turretController.stop();
    }

    private void handleControls() {
        // Start alignment
        if (gamepad1.dpad_up) {
            startAlignment();
            waitForButtonRelease();
        }

        // Stop alignment
        if (gamepad1.dpad_down) {
            stopAlignment();
            waitForButtonRelease();
        }

        // Change target tag ID
        if (gamepad1.dpad_left) {
            targetTagId--;
            if (targetTagId < 0) targetTagId = 0;
            alignmentController.setTargetTag(targetTagId);
            telemetry.addData("Target Tag", targetTagId);
            telemetry.update();
            sleep(200);
        }

        if (gamepad1.dpad_right) {
            targetTagId++;
            if (targetTagId > 30) targetTagId = 30;
            alignmentController.setTargetTag(targetTagId);
            telemetry.addData("Target Tag", targetTagId);
            telemetry.update();
            sleep(200);
        }

        // Toggle continuous mode
        if (gamepad1.a) {
            continuousMode = !continuousMode;
            telemetry.addData("Continuous Mode", continuousMode ? "ON" : "OFF");
            telemetry.update();
            waitForButtonRelease();
        }

        // Emergency stop
        if (gamepad1.b) {
            emergencyStop();
            waitForButtonRelease();
        }

        // Toggle detailed telemetry
        if (gamepad1.x) {
            detailedTelemetry = !detailedTelemetry;
            waitForButtonRelease();
        }

        // Reset and restart
        if (gamepad1.y) {
            restartAlignment();
            waitForButtonRelease();
        }
    }

    private void startAlignment() {
        alignmentAttempts++;
        alignmentController.startAlignment();
        telemetry.addLine("🎯 Starting alignment...");
        telemetry.update();
    }

    private void stopAlignment() {
        alignmentController.stopAlignment();
        telemetry.addLine("⏸ Alignment stopped");
        telemetry.update();
    }

    private void restartAlignment() {
        alignmentController.stopAlignment();
        sleep(100);
        startAlignment();
    }

    private void emergencyStop() {
        continuousMode = false;
        alignmentController.stopAlignment();
        turretController.stop();
        telemetry.addLine("🛑 EMERGENCY STOP");
        telemetry.update();
    }

    private void updateStatistics() {
        if (alignmentController.isAligned()) {
            double alignTime = alignmentController.getTotalAlignmentTime();

            if (alignTime > 0 && alignTime != totalAlignmentTime) {
                successfulAlignments++;
                totalAlignmentTime = alignTime;

                if (alignTime < bestAlignmentTime) {
                    bestAlignmentTime = alignTime;
                }
                if (alignTime > worstAlignmentTime) {
                    worstAlignmentTime = alignTime;
                }
            }
        }
    }

    private void displayStandardTelemetry() {
        telemetry.addLine("╔══════════════════════════╗");
        telemetry.addLine("║ TURRET ALIGNMENT TEST    ║");
        telemetry.addLine("╚══════════════════════════╝");
        telemetry.addLine();

        // Current state
        telemetry.addData("State", alignmentController.getState());
        telemetry.addData("Target Tag", targetTagId);
        telemetry.addData("Has Target", alignmentController.hasTarget() ? "✓" : "✗");
        telemetry.addLine();

        // Turret status
        telemetry.addData("Current Angle", "%.2f°", turretController.getCurrentAngle());
        telemetry.addData("Target Error", "%.2f°", alignmentController.getTargetError());
        telemetry.addData("Turret Power", "%.2f", turretController.getPower());
        telemetry.addLine();

        // Alignment status
        if (alignmentController.isAligned()) {
            telemetry.addLine("✓✓✓ ALIGNED ✓✓✓");
        } else if (alignmentController.isSearching()) {
            telemetry.addLine("🔍 SEARCHING...");
        }
        telemetry.addLine();

        // Test statistics
        telemetry.addLine("─── STATISTICS ───");
        telemetry.addData("Attempts", alignmentAttempts);
        telemetry.addData("Successful", successfulAlignments);
        if (alignmentAttempts > 0) {
            double successRate = (successfulAlignments * 100.0) / alignmentAttempts;
            telemetry.addData("Success Rate", "%.1f%%", successRate);
        }
        if (bestAlignmentTime < Double.MAX_VALUE) {
            telemetry.addData("Best Time", "%.2fs", bestAlignmentTime);
        }
        if (worstAlignmentTime > 0) {
            telemetry.addData("Worst Time", "%.2fs", worstAlignmentTime);
        }
        telemetry.addLine();

        // Mode indicators
        telemetry.addData("Continuous Mode", continuousMode ? "ON" : "OFF");
        telemetry.addData("Runtime", "%.1fs", runtime.seconds());
    }

    private void displayDetailedTelemetry() {
        telemetry.addLine("╔══════════════════════════╗");
        telemetry.addLine("║ DETAILED TELEMETRY       ║");
        telemetry.addLine("╚══════════════════════════╝");
        telemetry.addLine();

        // State information
        telemetry.addLine("─── STATE ───");
        telemetry.addData("State", alignmentController.getState());
        telemetry.addData("Target Tag", targetTagId);
        telemetry.addData("Has Target", alignmentController.hasTarget());
        telemetry.addLine();

        // Turret information
        telemetry.addLine("─── TURRET ───");
        telemetry.addData("Current", "%.2f°", turretController.getCurrentAngle());
        telemetry.addData("Error", "%.2f°", turretController.getAngleError());
        telemetry.addData("Power", "%.3f", turretController.getPower());
        telemetry.addData("At Target", turretController.atTarget());
        telemetry.addLine();

        // Alignment information
        telemetry.addLine("─── ALIGNMENT ───");
        telemetry.addData("Target Error", "%.2f°", alignmentController.getTargetError());
        telemetry.addData("Is Aligned", alignmentController.isAligned());
        telemetry.addData("Is Searching", alignmentController.isSearching());
        telemetry.addData("Total Time", "%.2fs", alignmentController.getTotalAlignmentTime());
        telemetry.addLine();

        // Parameters
        telemetry.addLine("─── PARAMETERS ───");
        telemetry.addData("Angle Tolerance", "%.1f°",
                TurretAlignmentController.AlignmentParams.ANGLE_TOLERANCE_DEGREES);
        telemetry.addData("Frames Required",
                TurretAlignmentController.AlignmentParams.ALIGNED_FRAMES_REQUIRED);
        telemetry.addData("Timeout", "%dms",
                TurretAlignmentController.AlignmentParams.ALIGNMENT_TIMEOUT_MS);
        telemetry.addLine();

        // Statistics
        telemetry.addLine("─── STATISTICS ───");
        telemetry.addData("Attempts", alignmentAttempts);
        telemetry.addData("Successful", successfulAlignments);
        if (alignmentAttempts > 0) {
            telemetry.addData("Success Rate", "%.1f%%",
                    (successfulAlignments * 100.0) / alignmentAttempts);
        }
        if (bestAlignmentTime < Double.MAX_VALUE) {
            telemetry.addData("Best", "%.2fs", bestAlignmentTime);
        }
        if (worstAlignmentTime > 0) {
            telemetry.addData("Worst", "%.2fs", worstAlignmentTime);
        }
        if (successfulAlignments > 0) {
            telemetry.addData("Average", "%.2fs",
                    totalAlignmentTime / successfulAlignments);
        }
    }

    private void displayControls() {
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  DPAD_UP: Start alignment");
        telemetry.addLine("  DPAD_DOWN: Stop alignment");
        telemetry.addLine("  DPAD_LEFT/RIGHT: Change tag ID");
        telemetry.addLine("  A: Toggle continuous mode");
        telemetry.addLine("  B: Emergency stop");
        telemetry.addLine("  X: Toggle detailed telemetry");
        telemetry.addLine("  Y: Reset & restart");
    }

    private void waitForButtonRelease() {
        while (opModeIsActive() &&
                (gamepad1.dpad_up || gamepad1.dpad_down ||
                        gamepad1.dpad_left || gamepad1.dpad_right ||
                        gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)) {
            sleep(10);
        }
    }
}