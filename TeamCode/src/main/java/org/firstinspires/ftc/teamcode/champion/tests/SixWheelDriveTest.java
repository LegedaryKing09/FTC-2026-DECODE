package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

/**
 * Comprehensive test for SixWheelDriveController
 * Tests motor connections, directions, and Pinpoint odometry
 */
@TeleOp(name = "SixWheel Drive Test", group = "Test")
public class SixWheelDriveTest extends LinearOpMode {

    private SixWheelDriveController driveController;
    private boolean testMode = false;
    private int testPhase = 0;
    private long phaseStartTime = 0;

    @Override
    public void runOpMode() {

        telemetry.addLine("=== INITIALIZING HARDWARE ===");
        telemetry.update();

        // Create controller instance
        driveController = new SixWheelDriveController(this);

        // Display startup info
        telemetry.clear();
        telemetry.addLine("=== HARDWARE TEST READY ===");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("• Left Stick Y: Forward/Backward");
        telemetry.addLine("• Right Stick X: Turn");
        telemetry.addLine("• A: Test Mode (cycles through motors)");
        telemetry.addLine("• B: Reset Odometry");
        telemetry.addLine("• X: Speed Mode Toggle");
        telemetry.addLine("• Y: Emergency Stop");
        telemetry.addLine();
        telemetry.addLine("Motor Status: " + driveController.getMotorStatus());
        telemetry.addLine("Pinpoint Status: " + driveController.getPinpointStatus());
        telemetry.addLine();
        telemetry.addLine(">>> Press START to begin <<<");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        phaseStartTime = System.currentTimeMillis();

        // Main control loop
        while (opModeIsActive()) {

            // Get gamepad inputs
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double turn = gamepad1.right_stick_x;   // Rotation

            // Button controls
            if (gamepad1.a && !testMode) {
                testMode = true;
                testPhase = 0;
                phaseStartTime = System.currentTimeMillis();
            }

            if (gamepad1.b) {
                driveController.resetOdometry();
                gamepad1.rumble(200);  // Haptic feedback
            }

            if (gamepad1.x) {
                if (driveController.isFastSpeedMode()) {
                    driveController.setSlowSpeed();
                } else {
                    driveController.setFastSpeed();
                }
                gamepad1.rumble(100);
            }

            if (gamepad1.y) {
                // Emergency stop
                driveController.stopDrive();
                testMode = false;
                gamepad1.rumble(500);
            }

            // Control logic
            if (testMode) {
                runTestSequence();
            } else {
                // Normal arcade drive
                driveController.arcadeDrive(drive, turn);
            }

            // Update odometry
            driveController.updateOdometry();

            // Display telemetry
            displayTelemetry(drive, turn);

            sleep(20);  // 50Hz update rate
        }

        // Cleanup
        driveController.stopDrive();
    }

    /**
     * Run automated test sequence
     */
    private void runTestSequence() {
        long elapsed = System.currentTimeMillis() - phaseStartTime;

        // Each phase lasts 2 seconds
        if (elapsed > 2000) {
            testPhase++;
            phaseStartTime = System.currentTimeMillis();

            if (testPhase > 7) {
                testMode = false;
                driveController.stopDrive();
                return;
            }
        }

        // Test different motor combinations
        switch (testPhase) {
            case 0:  // Test front left
                driveController.setFrontLeftPower(0.3);
                driveController.setFrontRightPower(0);
                driveController.setBackLeftPower(0);
                driveController.setBackRightPower(0);
                break;

            case 1:  // Test front right
                driveController.setFrontLeftPower(0);
                driveController.setFrontRightPower(0.3);
                driveController.setBackLeftPower(0);
                driveController.setBackRightPower(0);
                break;

            case 2:  // Test back left
                driveController.setFrontLeftPower(0);
                driveController.setFrontRightPower(0);
                driveController.setBackLeftPower(0.3);
                driveController.setBackRightPower(0);
                break;

            case 3:  // Test back right
                driveController.setFrontLeftPower(0);
                driveController.setFrontRightPower(0);
                driveController.setBackLeftPower(0);
                driveController.setBackRightPower(0.3);
                break;

            case 4:  // Test left side
                driveController.setLeftPower(0.3);
                driveController.setRightPower(0);
                break;

            case 5:  // Test right side
                driveController.setLeftPower(0);
                driveController.setRightPower(0.3);
                break;

            case 6:  // Test forward
                driveController.tankDrive(0.3, 0.3);
                break;

            case 7:  // Test rotation
                driveController.tankDrive(0.3, -0.3);
                break;
        }
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry(double drive, double turn) {
        telemetry.clear();

        // Mode indicator
        if (testMode) {
            telemetry.addLine("═══ TEST MODE ACTIVE ═══");
            telemetry.addData("Test Phase", getTestPhaseName());
            telemetry.addLine();
        } else {
            telemetry.addLine("═══ MANUAL CONTROL ═══");
            telemetry.addLine();
        }

        // Input values
        telemetry.addLine("INPUTS:");
        telemetry.addData("  Drive", "%.2f", drive);
        telemetry.addData("  Turn", "%.2f", turn);
        telemetry.addData("  Speed Mode", driveController.isFastSpeedMode() ? "FAST" : "SLOW");
        telemetry.addLine();

        // Motor status
        telemetry.addLine("MOTOR STATUS:");
        telemetry.addData("  Configuration", driveController.getMotorStatus());
        telemetry.addData("  Powers", driveController.getMotorPowers());
        telemetry.addLine();

        // Odometry data
        telemetry.addLine("ODOMETRY:");
        telemetry.addData("  X Position", "%.1f in", driveController.getX());
        telemetry.addData("  Y Position", "%.1f in", driveController.getY());
        telemetry.addData("  Heading", "%.1f°", driveController.getHeadingDegrees());
        telemetry.addLine();

        // Raw encoder values
        telemetry.addLine("RAW ENCODERS:");
        telemetry.addData("  X Encoder", driveController.getXOdoPosition());
        telemetry.addData("  Y Encoder", driveController.getYOdoPosition());
        telemetry.addLine();

        // Pinpoint status
        telemetry.addLine("PINPOINT:");
        telemetry.addData("  Status", driveController.getPinpointStatus());
        telemetry.addLine();

        // Control hints
        if (!testMode) {
            telemetry.addLine("Press A for test mode | B to reset odometry");
            telemetry.addLine("Press X for speed toggle | Y for emergency stop");
        } else {
            telemetry.addLine("Press Y to exit test mode");
        }

        telemetry.update();
    }

    /**
     * Get descriptive name for current test phase
     */
    private String getTestPhaseName() {
        switch (testPhase) {
            case 0: return "Front Left Motor";
            case 1: return "Front Right Motor";
            case 2: return "Back Left Motor";
            case 3: return "Back Right Motor";
            case 4: return "Left Side";
            case 5: return "Right Side";
            case 6: return "Forward Motion";
            case 7: return "Rotation";
            default: return "Complete";
        }
    }
}