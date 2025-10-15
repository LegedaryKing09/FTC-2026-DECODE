package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@TeleOp(name = "Odometry Test", group = "Testing")
public class OdometryTest extends LinearOpMode {

    private SixWheelDriveController driveController;
    private final ElapsedTime runtime = new ElapsedTime();
    private double startX, startY, startHeading;
    private boolean resetPressed = false;
    private boolean testModeActive = false;

    @Override
    public void runOpMode() {
        // Initialize the drive controller
        try {
            driveController = new SixWheelDriveController(this);
            telemetry.addData("Status", "Drive controller initialized successfully");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize drive controller: " + e.getMessage());
            telemetry.update();
            return;
        }

        // Wait for the Pinpoint to be ready
        telemetry.addData("Status", "Waiting for Pinpoint to be ready...");
        telemetry.update();

        while (!opModeIsActive()) {
            driveController.updateOdometry();
            GoBildaPinpointDriver.DeviceStatus status = driveController.getPinpointStatus();

            telemetry.addData("Pinpoint Status", status.toString());
            telemetry.addData("Raw Encoders", "X: %d, Y: %d",
                    driveController.getXOdoPosition(),
                    driveController.getYOdoPosition());
            //*telemetry.addData("Pinpoint Info", driveController.getPinpointTelemetry());
            telemetry.addData("", "");
            telemetry.addData("Controls", "Press PLAY when ready");
            telemetry.update();

            sleep(100);
        }

        // Record starting position
        driveController.updateOdometry();
        startX = driveController.getX();
        startY = driveController.getY();
        startHeading = driveController.getHeadingDegrees();

        telemetry.addData("Status", "Initialized and ready!");
        telemetry.addData("Starting Position", "X: %.1f, Y: %.1f, H: %.1f°",
                startX, startY, startHeading);
        telemetry.update();

        runtime.reset();

        // Main loop
        while (opModeIsActive()) {
            // Update odometry every loop
            driveController.updateOdometry();

            // Handle controller input
            handleControls();

            // Display telemetry
            displayTelemetry();

            // Small delay to prevent overwhelming the system
            sleep(10);
        }
    }

    private void handleControls() {
        // Drive controls
        double drive = -gamepad1.left_stick_y;  // Forward/backward
        double turn = gamepad1.right_stick_x;   // Turn left/right

        // Apply drive power
        driveController.arcadeDrive(drive * 0.7, turn * 0.5);

        // Reset odometry
        if (gamepad1.y && !resetPressed) {
            driveController.resetOdometry();
            startX = 0;
            startY = 0;
            startHeading = 0;
            resetPressed = true;
            telemetry.addLine("ODOMETRY RESET!");
        } else if (!gamepad1.y) {
            resetPressed = false;
        }

        // Toggle test mode
        if (gamepad1.x) {
            testModeActive = !testModeActive;
            if (testModeActive) {
                // Start automated test sequence
                performAutomatedTest();
            }
            while (gamepad1.x) {
                // Wait for button release
                sleep(10);
            }
        }

        // Manual motor test (for debugging)
        if (gamepad1.dpad_up) {
            driveController.setAllMotorPower(0.3);
        } else if (gamepad1.dpad_down) {
            driveController.setAllMotorPower(-0.3);
        } else if (gamepad1.dpad_left) {
            driveController.setLeftPower(0.3);
            driveController.setRightPower(-0.3);
        } else if (gamepad1.dpad_right) {
            driveController.setLeftPower(-0.3);
            driveController.setRightPower(0.3);
        } else if (!gamepad1.a && !gamepad1.b) {
            // Only stop if not using drive sticks and not in other test modes
            if (Math.abs(drive) < 0.1 && Math.abs(turn) < 0.1) {
                driveController.stopDrive();
            }
        }
    }

    private void performAutomatedTest() {
        telemetry.addLine("Starting automated test sequence...");
        telemetry.update();

        // Record starting position
        double testStartX = driveController.getX();
        double testStartY = driveController.getY();
        double testStartHeading = driveController.getHeadingDegrees();

        // Test 1: Drive forward
        telemetry.addLine("Test 1: Driving forward for 2 seconds");
        telemetry.update();
        driveController.arcadeDrive(0.5, 0);
        sleep(2000);
        driveController.stopDrive();

        driveController.updateOdometry();
        double forwardX = driveController.getX();
        double forwardY = driveController.getY();

        telemetry.addData("Forward Test Result", "ΔX: %.1f, ΔY: %.1f",
                forwardX - testStartX, forwardY - testStartY);
        telemetry.update();
        sleep(1000);

        // Test 2: Turn in place
        telemetry.addLine("Test 2: Turning in place for 2 seconds");
        telemetry.update();
        driveController.arcadeDrive(0, 0.5);
        sleep(2000);
        driveController.stopDrive();

        driveController.updateOdometry();
        double turnHeading = driveController.getHeadingDegrees();

        telemetry.addData("Turn Test Result", "ΔHeading: %.1f°",
                turnHeading - testStartHeading);
        telemetry.update();
        sleep(1000);

        // Test 3: Drive backward to start
        telemetry.addLine("Test 3: Returning to start position");
        telemetry.update();
        driveController.arcadeDrive(-0.5, 0);
        sleep(2000);
        driveController.stopDrive();

        driveController.updateOdometry();
        double finalX = driveController.getX();
        double finalY = driveController.getY();
        double finalHeading = driveController.getHeadingDegrees();

        telemetry.addData("Final Position vs Start",
                "ΔX: %.1f, ΔY: %.1f, ΔH: %.1f°",
                finalX - testStartX,
                finalY - testStartY,
                finalHeading - testStartHeading);
        telemetry.addLine("Automated test complete!");
        telemetry.update();

        testModeActive = false;
    }

    private void displayTelemetry() {
        // Runtime
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addLine();

        // Pinpoint status
        telemetry.addData("Pinpoint Status", driveController.getPinpointStatus().toString());
        telemetry.addData("Pinpoint Info", driveController.getPinpointTelemetry());
        telemetry.addLine();

        // Current position
        telemetry.addData("Current Position",
                "X: %.1f in, Y: %.1f in",
                driveController.getX(),
                driveController.getY());
        telemetry.addData("Current Heading", "%.1f° (%.3f rad)",
                driveController.getHeadingDegrees(),
                driveController.getHeading());
        telemetry.addLine();

        // Distance traveled from start
        double deltaX = driveController.getX() - startX;
        double deltaY = driveController.getY() - startY;
        double deltaHeading = driveController.getHeadingDegrees() - startHeading;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        telemetry.addData("Change from Start",
                "ΔX: %.1f, ΔY: %.1f, ΔH: %.1f°",
                deltaX, deltaY, deltaHeading);
        telemetry.addData("Total Distance", "%.1f in",
                distance);
        telemetry.addLine();

        // Velocities
        telemetry.addData("Velocities",
                "X: %.1f mm/s, Y: %.1f mm/s, H: %.1f°/s",
                driveController.getVelocityX(),
                driveController.getVelocityY(),
                Math.toDegrees(driveController.getHeadingVelocity()));
        telemetry.addLine();

        // Raw encoder values
        telemetry.addData("Raw Encoders",
                "X: %d ticks, Y: %d ticks",
                driveController.getXOdoPosition(),
                driveController.getYOdoPosition());
        telemetry.addLine();

        // Motor status and powers
        telemetry.addData("Motor Status", driveController.getMotorStatus());
        telemetry.addData("Motor Powers", driveController.getMotorPowers());
        telemetry.addLine();

        // Controls help
        telemetry.addData("Controls", "Left stick: drive, Right stick: turn");
        telemetry.addData("", "Y: Reset odometry, X: Auto test");
        telemetry.addData("", "DPad: Manual motor test");

        if (testModeActive) {
            telemetry.addLine();
            telemetry.addData("STATUS", "AUTOMATED TEST RUNNING...");
        }

        telemetry.update();
    }
}
