package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.PoseStorage;

/**
 * Odometry Accuracy Test
 *
 * Tests if getPose() returns correct field coordinates.
 *
 * === TEST PROCEDURE ===
 *
 * TEST 1: Starting Position
 * 1. Place robot at a KNOWN position on field (e.g., corner at x=0, y=0, heading=0°)
 * 2. Press START
 * 3. Check if displayed position matches actual position
 *
 * TEST 2: Drive Forward
 * 1. Start at known position
 * 2. Drive forward exactly 24 inches (use tape measure)
 * 3. Check if Y increased by ~24
 *
 * TEST 3: Strafe/Turn
 * 1. Turn 90° using field tiles as reference
 * 2. Check if heading changed by ~90°
 *
 * TEST 4: Square Test
 * 1. Drive a 24" x 24" square back to start
 * 2. Check if final position matches start
 *
 * === CONTROLS ===
 * Left Stick:    Drive forward/backward
 * Right Stick:   Turn
 *
 * A: Reset pose to (0, 0, 0°)
 * B: Reset pose to PoseStorage (from auton)
 * X: Set current position as "checkpoint"
 * Y: Show error from checkpoint
 *
 * Dpad Up:    Set pose to (0, 24, 0°) - 24" forward
 * Dpad Down:  Set pose to (0, -24, 0°) - 24" backward
 * Dpad Left:  Set pose to (-24, 0, 0°) - 24" left
 * Dpad Right: Set pose to (24, 0, 0°) - 24" right
 */
@Config
@TeleOp(name = "Odometry Accuracy Test", group = "Test")
public class OdometryAccuracy extends LinearOpMode {

    // Tunable via Dashboard
    public static double DRIVE_SPEED = 0.5;
    public static double TURN_SPEED = 0.4;

    private SixWheelDriveController drive;
    private FtcDashboard dashboard;

    // Checkpoint for error measurement
    private double checkpointX = 0;
    private double checkpointY = 0;
    private double checkpointHeading = 0;
    private boolean checkpointSet = false;

    // Button states
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize drive
        telemetry.addLine("Initializing drive...");
        telemetry.update();

        try {
            drive = new SixWheelDriveController(this);
            telemetry.addLine("✓ Drive initialized");
        } catch (Exception e) {
            telemetry.addLine("✗ Drive FAILED: " + e.getMessage());
            telemetry.update();
            return;
        }

        // Show PoseStorage value
        Pose2d storedPose = PoseStorage.currentPose;
        telemetry.addLine("");
        telemetry.addLine("=== POSE STORAGE ===");
        telemetry.addData("Stored X", "%.2f", storedPose.position.x);
        telemetry.addData("Stored Y", "%.2f", storedPose.position.y);
        telemetry.addData("Stored Heading", "%.2f°", Math.toDegrees(storedPose.heading.toDouble()));

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Reset to (0,0,0)");
        telemetry.addLine("B: Load from PoseStorage");
        telemetry.addLine("X: Set checkpoint");
        telemetry.addLine("Y: Show error from checkpoint");
        telemetry.addLine("");
        telemetry.addLine("Place robot at KNOWN position");
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        // Initialize odometry
        drive.updateOdometry();

        telemetry.addLine("=== INITIAL READING ===");
        telemetry.addData("X", "%.2f", drive.getX());
        telemetry.addData("Y", "%.2f", drive.getY());
        telemetry.addData("Heading", "%.2f°", drive.getHeadingDegrees());
        telemetry.update();
        sleep(1000);

        while (opModeIsActive()) {
            // Update odometry
            drive.updateOdometry();

            // Handle driving
            double forward = -gamepad1.left_stick_y * DRIVE_SPEED;
            double turn = gamepad1.right_stick_x * TURN_SPEED;
            drive.arcadeDrive(forward, turn);

            // Handle button presses
            handleButtons();

            // Update telemetry
            updateTelemetry();

            // Send to dashboard
            sendDashboardData();
        }
    }

    private void handleButtons() {
        // A: Reset to origin
        if (gamepad1.a && !lastA) {
            drive.setPosition(0, 0, 0);
            drive.updateOdometry();
            telemetry.addLine(">>> RESET TO (0, 0, 0°) <<<");
        }
        lastA = gamepad1.a;

        // B: Load from PoseStorage
        if (gamepad1.b && !lastB) {
            Pose2d stored = PoseStorage.currentPose;
            drive.setPosition(
                    stored.position.x,
                    stored.position.y,
                    stored.heading.toDouble()
            );
            drive.updateOdometry();
            telemetry.addLine(">>> LOADED FROM POSESTORAGE <<<");
        }
        lastB = gamepad1.b;

        // X: Set checkpoint
        if (gamepad1.x && !lastX) {
            checkpointX = drive.getX();
            checkpointY = drive.getY();
            checkpointHeading = drive.getHeadingDegrees();
            checkpointSet = true;
            telemetry.addLine(">>> CHECKPOINT SET <<<");
        }
        lastX = gamepad1.x;

        // Y: Show error (calculated in telemetry)
        lastY = gamepad1.y;

        // Dpad: Set to known positions for testing
        if (gamepad1.dpad_up && !lastDpadUp) {
            drive.setPosition(0, 24, 0);
            drive.updateOdometry();
            telemetry.addLine(">>> SET TO (0, 24, 0°) <<<");
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            drive.setPosition(0, -24, 0);
            drive.updateOdometry();
            telemetry.addLine(">>> SET TO (0, -24, 0°) <<<");
        }
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_left && !lastDpadLeft) {
            drive.setPosition(-24, 0, 0);
            drive.updateOdometry();
            telemetry.addLine(">>> SET TO (-24, 0, 0°) <<<");
        }
        lastDpadLeft = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !lastDpadRight) {
            drive.setPosition(24, 0, 0);
            drive.updateOdometry();
            telemetry.addLine(">>> SET TO (24, 0, 0°) <<<");
        }
        lastDpadRight = gamepad1.dpad_right;
    }

    private void updateTelemetry() {
        telemetry.addLine("=== CURRENT POSE ===");
        telemetry.addData("X", "%.2f inches", drive.getX());
        telemetry.addData("Y", "%.2f inches", drive.getY());
        telemetry.addData("Heading", "%.2f°", drive.getHeadingDegrees());

        // Raw heading in radians for debugging
        telemetry.addData("Heading (rad)", "%.4f", Math.toRadians(drive.getHeadingDegrees()));

        telemetry.addLine("");
        telemetry.addLine("=== POSE STORAGE ===");
        Pose2d stored = PoseStorage.currentPose;
        telemetry.addData("Stored", "(%.1f, %.1f, %.1f°)",
                stored.position.x, stored.position.y, Math.toDegrees(stored.heading.toDouble()));

        // Checkpoint error
        if (checkpointSet) {
            telemetry.addLine("");
            telemetry.addLine("=== CHECKPOINT ERROR ===");
            telemetry.addData("Checkpoint", "(%.1f, %.1f, %.1f°)",
                    checkpointX, checkpointY, checkpointHeading);

            double errorX = drive.getX() - checkpointX;
            double errorY = drive.getY() - checkpointY;
            double errorH = drive.getHeadingDegrees() - checkpointHeading;
            // Normalize heading error
            while (errorH > 180) errorH -= 360;
            while (errorH < -180) errorH += 360;

            double errorDistance = Math.sqrt(errorX * errorX + errorY * errorY);

            telemetry.addData("Error X", "%.2f inches", errorX);
            telemetry.addData("Error Y", "%.2f inches", errorY);
            telemetry.addData("Error Distance", "%.2f inches", errorDistance);
            telemetry.addData("Error Heading", "%.2f°", errorH);
        }

        telemetry.addLine("");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A=Reset(0,0) B=LoadStorage");
        telemetry.addLine("X=Checkpoint Y=ShowError");
        telemetry.addLine("Dpad=Set known positions");

        telemetry.update();
    }

    private void sendDashboardData() {
        TelemetryPacket packet = new TelemetryPacket();

        // Position data
        packet.put("Odom/X", drive.getX());
        packet.put("Odom/Y", drive.getY());
        packet.put("Odom/Heading", drive.getHeadingDegrees());

        // Draw robot on field
        double x = drive.getX();
        double y = drive.getY();
        double heading = Math.toRadians(drive.getHeadingDegrees());

        // Robot as a small triangle
        double size = 9;  // Robot size in inches
        packet.fieldOverlay()
                .setFill("blue")
                .setStroke("white")
                .strokeCircle(x, y, size/2)
                .strokeLine(x, y,
                        x + size * Math.sin(heading),
                        y + size * Math.cos(heading));

        // Draw target at origin
        packet.fieldOverlay()
                .setFill("red")
                .fillCircle(0, 0, 3);

        // Draw checkpoint if set
        if (checkpointSet) {
            packet.fieldOverlay()
                    .setFill("green")
                    .fillCircle(checkpointX, checkpointY, 2);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}