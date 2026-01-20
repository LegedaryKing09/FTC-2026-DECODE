package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

/**
 * Comprehensive Odometry Test OpMode6
 *
 * Controls:
 * - DPAD_UP: Start/Stop static drift test
 * - DPAD_DOWN: Reset odometry to zero
 * - DPAD_LEFT: Start square test (manual drive)
 * - DPAD_RIGHT: Start spin test
 * - A: Mark current position (for measuring known distances)
 * - B: Calculate distance from marked position
 * - X: Toggle between inches and mm display
 * - Y: Recalibrate/reset Pinpoint
 * - Left Stick: Manual drive (for push tests)
 * 1. Static Drift Test (DPAD_UP)
 *
 * Place robot still on the ground
 * Press DPAD_UP to start
 * Wait 10 seconds
 * Check if X, Y, or heading drifts while stationary
 * If you see drift: Your encoders or IMU have noise/issues
 *
 * 2. Known Distance Test (A + B buttons)
 *
 * Reset odometry (DPAD_DOWN)
 * Press A to mark position
 * Physically push the robot exactly 24 inches forward (use a tape measure)
 * Press B to calculate distance
 * Compare measured vs actual
 * If off by >5%: Your encoder resolution or wheel diameter settings may be wrong
 *
 * 3. Square Return Test
 *
 * Reset odometry (DPAD_DOWN)
 * Push robot in a square (24" x 24")
 * Return to starting point
 * Check if X and Y are close to 0
 * If not: Heading integration error or encoder scaling issue
 *
 * 4. Spin Test
 *
 * Reset odometry (DPAD_DOWN)
 * Mark position (A)
 * Rotate robot exactly 360° by hand
 * Check heading - should return to ~0°
 * If off: Your YAW_SCALAR in OdometryParams needs adjustment
 */
@Config
@TeleOp(name = "Odometry Test", group = "Testing")
public class OdometryTest extends LinearOpMode {

    // Test configuration
    @Config
    public static class OdoTestConfig {
        public static double STATIC_TEST_DURATION_SEC = 10.0;
        public static double DRIFT_THRESHOLD_INCHES = 0.1;
        public static double DRIFT_THRESHOLD_DEGREES = 0.5;
        public static double EXPECTED_SQUARE_SIDE_INCHES = 24.0;
    }

    private SixWheelDriveController driveController;
    private final ElapsedTime timer = new ElapsedTime();

    // Test state
    private boolean staticTestRunning = false;
    private double staticTestStartX, staticTestStartY, staticTestStartHeading;
    private double maxDriftX, maxDriftY, maxDriftHeading;

    // Marked position for distance measurement
    private double markedX, markedY, markedHeading;
    private boolean positionMarked = false;

    // Display mode
    private boolean showInches = true;

    // Test results storage
    private StringBuilder testLog = new StringBuilder();

    @Override
    public void runOpMode() {
        // Initialize
        driveController = new SixWheelDriveController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== ODOMETRY TEST INITIALIZED ===");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry every loop
            driveController.updateOdometry();

            // Handle controls
            handleControls();

            // Run active tests
            if (staticTestRunning) {
                runStaticDriftTest();
            }

            // Display telemetry
            displayTelemetry();

            sleep(20);
        }
    }

    private void handleControls() {
        // DPAD_UP: Toggle static drift test
        if (gamepad1.dpad_up) {
            if (!staticTestRunning) {
                startStaticDriftTest();
            } else {
                stopStaticDriftTest();
            }
            sleep(300);
        }

        // DPAD_DOWN: Reset odometry
        if (gamepad1.dpad_down) {
            driveController.resetOdometry();
            positionMarked = false;
            testLog.append("Odometry RESET\n");
            sleep(300);
        }

        // A: Mark current position
        if (gamepad1.a) {
            markPosition();
            sleep(300);
        }

        // B: Calculate distance from marked position
        if (gamepad1.b && positionMarked) {
            calculateDistanceFromMark();
            sleep(300);
        }

        // X: Toggle units
        if (gamepad1.x) {
            showInches = !showInches;
            sleep(300);
        }

        // Y: Recalibrate Pinpoint
        if (gamepad1.y) {
            telemetry.addLine("⏳ Recalibrating Pinpoint...");
            telemetry.update();
            driveController.resetOdometry();
            sleep(500);
            testLog.append("Pinpoint RECALIBRATED\n");
            sleep(300);
        }

        // Manual drive for push tests
        double drive = -gamepad1.left_stick_y * 0.5;
        double turn = gamepad1.right_stick_x * 0.3;
        if (Math.abs(drive) > 0.05 || Math.abs(turn) > 0.05) {
            driveController.arcadeDrive(drive, turn);
        } else {
            driveController.stopDrive();
        }
    }

    // ==================== STATIC DRIFT TEST ====================
    private void startStaticDriftTest() {
        staticTestRunning = true;
        timer.reset();

        // Record starting position
        staticTestStartX = driveController.getX();
        staticTestStartY = driveController.getY();
        staticTestStartHeading = driveController.getHeadingDegrees();

        maxDriftX = 0;
        maxDriftY = 0;
        maxDriftHeading = 0;

        testLog.append("\n=== STATIC DRIFT TEST STARTED ===\n");
        testLog.append(String.format("Start: X=%.3f, Y=%.3f, H=%.2f°\n",
                staticTestStartX, staticTestStartY, staticTestStartHeading));
    }

    private void runStaticDriftTest() {
        double currentX = driveController.getX();
        double currentY = driveController.getY();
        double currentHeading = driveController.getHeadingDegrees();

        double driftX = Math.abs(currentX - staticTestStartX);
        double driftY = Math.abs(currentY - staticTestStartY);
        double driftHeading = Math.abs(currentHeading - staticTestStartHeading);

        // Track max drift
        maxDriftX = Math.max(maxDriftX, driftX);
        maxDriftY = Math.max(maxDriftY, driftY);
        maxDriftHeading = Math.max(maxDriftHeading, driftHeading);

        // Auto-stop after duration
        if (timer.seconds() >= OdoTestConfig.STATIC_TEST_DURATION_SEC) {
            stopStaticDriftTest();
        }
    }

    private void stopStaticDriftTest() {
        staticTestRunning = false;

        String result;
        boolean passed = maxDriftX < OdoTestConfig.DRIFT_THRESHOLD_INCHES &&
                maxDriftY < OdoTestConfig.DRIFT_THRESHOLD_INCHES &&
                maxDriftHeading < OdoTestConfig.DRIFT_THRESHOLD_DEGREES;

        result = String.format(
                "=== STATIC TEST %s ===\n" +
                        "Duration: %.1fs\n" +
                        "Max X Drift: %.4f in (threshold: %.2f)\n" +
                        "Max Y Drift: %.4f in (threshold: %.2f)\n" +
                        "Max Heading Drift: %.3f° (threshold: %.1f°)\n",
                passed ? "PASSED ✓" : "FAILED ✗",
                timer.seconds(),
                maxDriftX, OdoTestConfig.DRIFT_THRESHOLD_INCHES,
                maxDriftY, OdoTestConfig.DRIFT_THRESHOLD_INCHES,
                maxDriftHeading, OdoTestConfig.DRIFT_THRESHOLD_DEGREES
        );

        testLog.append(result);
    }

    // ==================== POSITION MARKING ====================
    private void markPosition() {
        markedX = driveController.getX();
        markedY = driveController.getY();
        markedHeading = driveController.getHeadingDegrees();
        positionMarked = true;

        testLog.append(String.format("\n📍 POSITION MARKED: X=%.3f, Y=%.3f, H=%.2f°\n",
                markedX, markedY, markedHeading));
    }

    private void calculateDistanceFromMark() {
        double currentX = driveController.getX();
        double currentY = driveController.getY();
        double currentHeading = driveController.getHeadingDegrees();

        double deltaX = currentX - markedX;
        double deltaY = currentY - markedY;
        double deltaHeading = currentHeading - markedHeading;

        // Normalize heading delta
        while (deltaHeading > 180) deltaHeading -= 360;
        while (deltaHeading < -180) deltaHeading += 360;

        double straightLineDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        String result = String.format(
                "\n=== DISTANCE FROM MARK ===\n" +
                        "ΔX: %.3f in\n" +
                        "ΔY: %.3f in\n" +
                        "Straight Line: %.3f in\n" +
                        "ΔHeading: %.2f°\n",
                deltaX, deltaY, straightLineDistance, deltaHeading
        );

        testLog.append(result);
    }

    // ==================== TELEMETRY ====================
    private void displayTelemetry() {
        telemetry.addLine("══════════════════════════════════");
        telemetry.addLine("       ODOMETRY TEST SUITE");
        telemetry.addLine("══════════════════════════════════");

        // Current position
        double x = driveController.getX();
        double y = driveController.getY();
        double heading = driveController.getHeadingDegrees();

        if (showInches) {
            telemetry.addData("📍 X Position", "%.3f inches", x);
            telemetry.addData("📍 Y Position", "%.3f inches", y);
        } else {
            telemetry.addData("📍 X Position", "%.2f mm", x * 25.4);
            telemetry.addData("📍 Y Position", "%.2f mm", y * 25.4);
        }
        telemetry.addData("🧭 Heading", "%.2f°", heading);

        telemetry.addLine();

        // Raw encoder values (critical for debugging)
        telemetry.addLine("=== RAW ENCODER DATA ===");
        telemetry.addData("X Encoder (ticks)", driveController.getXOdoPosition());
        telemetry.addData("Y Encoder (ticks)", driveController.getYOdoPosition());

        telemetry.addLine();

        // Velocities
        telemetry.addLine("=== VELOCITIES ===");
        telemetry.addData("Vel X", "%.2f in/s", driveController.getVelocityX());
        telemetry.addData("Vel Y", "%.2f in/s", driveController.getVelocityY());
        telemetry.addData("Heading Vel", "%.2f rad/s", driveController.getHeadingVelocity());

        telemetry.addLine();

        // Pinpoint status
        telemetry.addLine("=== PINPOINT STATUS ===");
        telemetry.addData("Device Status", driveController.getPinpointStatus());
        telemetry.addData("Loop Time", "%d μs", driveController.getPinpointLoopTime());
        telemetry.addData("Frequency", "%.1f Hz", driveController.getPinpointFrequency());

        telemetry.addLine();

        // Static test status
        if (staticTestRunning) {
            telemetry.addLine("🔴 STATIC DRIFT TEST RUNNING");
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), OdoTestConfig.STATIC_TEST_DURATION_SEC);
            telemetry.addData("Current X Drift", "%.4f in", Math.abs(driveController.getX() - staticTestStartX));
            telemetry.addData("Current Y Drift", "%.4f in", Math.abs(driveController.getY() - staticTestStartY));
            telemetry.addData("Current Heading Drift", "%.3f°", Math.abs(driveController.getHeadingDegrees() - staticTestStartHeading));
        }

        // Marked position
        if (positionMarked) {
            telemetry.addLine();
            telemetry.addLine("📌 MARKED POSITION:");
            telemetry.addData("  Mark X", "%.3f", markedX);
            telemetry.addData("  Mark Y", "%.3f", markedY);
            telemetry.addData("  Mark Heading", "%.2f°", markedHeading);

            double dist = Math.sqrt(Math.pow(x - markedX, 2) + Math.pow(y - markedY, 2));
            telemetry.addData("  Distance from mark", "%.3f in", dist);
        }

        telemetry.addLine();
        telemetry.addLine("══════════ CONTROLS ══════════");
        telemetry.addLine("DPAD_UP: Static drift test");
        telemetry.addLine("DPAD_DOWN: Reset odometry");
        telemetry.addLine("A: Mark position");
        telemetry.addLine("B: Calc distance from mark");
        telemetry.addLine("X: Toggle in/mm");
        telemetry.addLine("Y: Recalibrate Pinpoint");
        telemetry.addLine("Left Stick: Manual drive");

        // Show recent test log
        if (testLog.length() > 0) {
            telemetry.addLine();
            telemetry.addLine("=== RECENT TEST LOG ===");
            // Show last 500 chars of log
            String log = testLog.toString();
            if (log.length() > 500) {
                log = "..." + log.substring(log.length() - 500);
            }
            telemetry.addLine(log);
        }

        telemetry.update();
    }
}