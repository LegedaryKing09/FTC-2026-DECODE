package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

/**
 * ============================================================================
 * ODOMETRY BASIC TEST - NO PID
 * ============================================================================
 *
 * PURPOSE:
 * - Test if odometry is tracking position correctly after encoder changes
 * - Verify robot moves forward/backward using simple power control
 * - Check if reported distance matches actual distance traveled
 * - NO PID - just basic movement to test odometry accuracy
 *
 * RECENT CHANGES TO TEST:
 * - Changed from SWINGARM_POD to 4_BAR_POD
 * - Added custom encoder resolution: 19.9847352f mm
 *
 * HOW TO USE:
 * 1. Place robot in clear area (at least 8 feet straight line)
 * 2. Mark starting position with tape
 * 3. Deploy this OpMode
 * 4. Use controls to test movements
 * 5. Measure ACTUAL distance with tape measure
 * 6. Compare to odometry reading
 *
 * CONTROLS:
 * - DPAD UP: Move forward 24 inches (simple power)
 * - DPAD DOWN: Move backward 24 inches (simple power)
 * - X: Move forward 48 inches (long test)
 * - A: Move forward 12 inches (short test)
 * - B: Move backward 48 inches (long test)
 * - Y: Move backward 12 inches (short test)
 * - LEFT BUMPER: Reset odometry to (0, 0, 0°)
 * - RIGHT BUMPER: Run distance verification test
 * - START: Emergency stop
 *
 * WHAT TO LOOK FOR:
 * - Does odometry reading match actual distance? (within 5%)
 * - Does robot drive straight? (check heading stays near 0°)
 * - Is movement smooth and consistent?
 */
@Config
@TeleOp(name = "Odometry Basic Test (No PID)", group = "Test")
public class BasicOdometryTest extends LinearOpMode {

    // Movement parameters (tunable via dashboard)
    public static double FORWARD_POWER = 0.5;      // Power for forward movement
    public static double BACKWARD_POWER = -0.5;    // Power for backward movement
    public static double MOVEMENT_TIMEOUT_MS = 10000;  // Max time for any movement

    // Distance tracking
    public static double DISTANCE_TOLERANCE = 1.0;  // Stop within 1 inch of target

    private SixWheelDriveController driveController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();

    // Test tracking
    private double lastTestStartX = 0;
    private double lastTestStartY = 0;
    private double lastTestTargetDistance = 0;
    private double lastTestActualDistance = 0;
    private double lastTestDuration = 0;
    private String lastTestName = "None";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize drive controller
        driveController = new SixWheelDriveController(this);
        driveController.resetOdometry();

        displayInstructions();
        telemetry.addLine();
        telemetry.addLine("✓ Ready! Press START");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.clear();
        telemetry.addLine("STARTED");
        telemetry.addLine("Use DPAD to test movements");
        telemetry.update();

        while (opModeIsActive()) {
            driveController.updateOdometry();

            // Handle controls
            handleControls();

            // Display status
            displayStatus();

            sleep(20);
        }

        driveController.stopDrive();
    }

    private void handleControls() {
        // Emergency stop
        if (gamepad1.start) {
            driveController.stopDrive();
            telemetry.addLine("EMERGENCY STOP");
            telemetry.update();
            sleep(500);
            return;
        }

        // Reset odometry
        if (gamepad1.left_bumper) {
            driveController.resetOdometry();
            telemetry.addLine("Odometry reset to (0, 0, 0°)");
            telemetry.update();
            sleep(500);
            return;
        }

        // Run verification test
        if (gamepad1.right_bumper) {
            runVerificationTest();
            return;
        }

        // Movement tests
        if (gamepad1.dpad_up) {
            testSimpleMovement(24.0, "Forward 24 inches");
            sleep(500);
        } else if (gamepad1.dpad_down) {
            testSimpleMovement(-24.0, "Backward 24 inches");
            sleep(500);
        } else if (gamepad1.x) {
            testSimpleMovement(48.0, "Forward 48 inches (long)");
            sleep(500);
        } else if (gamepad1.a) {
            testSimpleMovement(12.0, "Forward 12 inches (short)");
            sleep(500);
        } else if (gamepad1.b) {
            testSimpleMovement(-48.0, "Backward 48 inches (long)");
            sleep(500);
        } else if (gamepad1.y) {
            testSimpleMovement(-12.0, "Backward 12 inches (short)");
            sleep(500);
        }
    }

    /**
     * Simple movement test - uses constant power until target distance reached
     * NO PID - just basic distance tracking
     */
    private void testSimpleMovement(double targetDistance, String testName) {
        telemetry.clear();
        telemetry.addLine();
        telemetry.addLine("Using simple power control");
        telemetry.addLine("NO PID - just odometry tracking");
        telemetry.update();

        // Record start position
        driveController.updateOdometry();
        lastTestStartX = driveController.getX();
        lastTestStartY = driveController.getY();
        lastTestTargetDistance = Math.abs(targetDistance);
        lastTestName = testName;

        moveTimer.reset();

        // Determine direction and power
        double direction = Math.signum(targetDistance);
        double power = (direction > 0) ? FORWARD_POWER : BACKWARD_POWER;

        // Set drive mode to POWER (not velocity)
        driveController.setDriveMode(SixWheelDriveController.DriveMode.POWER);

        // Move until target distance reached
        while (opModeIsActive() && moveTimer.milliseconds() < MOVEMENT_TIMEOUT_MS) {
            driveController.updateOdometry();

            // Calculate distance traveled
            double currentDistance = Math.abs(driveController.getX() - lastTestStartX);
            double remainingDistance = Math.abs(targetDistance) - currentDistance;

            // Check if target reached
            if (remainingDistance <= DISTANCE_TOLERANCE) {
                break;
            }

            // Apply power (tank drive)
            driveController.tankDrive(power, power);

            // Display progress
            telemetry.clear();
            telemetry.addLine("🔄 MOVING...");
            telemetry.addData("Target", "%.1f in", Math.abs(targetDistance));
            telemetry.addData("Current", "%.1f in", currentDistance);
            telemetry.addData("Remaining", "%.1f in", remainingDistance);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Time", "%.1f sec", moveTimer.seconds());
            telemetry.update();

            sleep(10);
        }

        // Stop
        driveController.stopDrive();

        // Record results
        driveController.updateOdometry();
        lastTestActualDistance = Math.abs(driveController.getX() - lastTestStartX);
        lastTestDuration = moveTimer.seconds();

        // Display results
        displayTestResults();
    }

    /**
     * Display test results with pass/fail analysis
     */
    private void displayTestResults() {
        double error = Math.abs(lastTestActualDistance - lastTestTargetDistance);
        double errorPercent = (error / lastTestTargetDistance) * 100;

        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║  TEST COMPLETE                ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();
        telemetry.addData("Test", lastTestName);
        telemetry.addData("Target Distance", "%.1f in", lastTestTargetDistance);
        telemetry.addData("Odometry Reading", "%.1f in", lastTestActualDistance);
        telemetry.addData("Error", "%.1f in (%.1f%%)", error, errorPercent);
        telemetry.addData("Duration", "%.1f sec", lastTestDuration);
        telemetry.addLine();

        // Analysis
        telemetry.addLine("═══ ANALYSIS ═══");
        if (errorPercent < 5.0) {
            telemetry.addLine("✅ EXCELLENT - Odometry accurate!");
        } else if (errorPercent < 10.0) {
            telemetry.addLine("⚠️ ACCEPTABLE - Minor error");
        } else if (errorPercent < 20.0) {
            telemetry.addLine("⚠️ NEEDS ATTENTION");
            telemetry.addLine("Check:");
            telemetry.addLine("  • Encoder directions");
            telemetry.addLine("  • Pod offsets");
            telemetry.addLine("  • Wheel slippage");
        } else {
            telemetry.addLine("❌ SIGNIFICANT ERROR");
            telemetry.addLine("Odometry may be misconfigured!");
            telemetry.addLine("Check:");
            telemetry.addLine("  • Encoder resolution setting");
            telemetry.addLine("  • Encoder directions (X/Y)");
            telemetry.addLine("  • Pod offset measurements");
            telemetry.addLine("  • Physical pod alignment");
        }
        telemetry.addLine();
        telemetry.addLine("⚠️ IMPORTANT: Measure actual");
        telemetry.addLine("distance with tape measure!");

        telemetry.update();
        sleep(3000);
    }

    /**
     * Run comprehensive distance verification test
     * Tests multiple distances and provides detailed analysis
     */
    private void runVerificationTest() {
        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║  DISTANCE VERIFICATION TEST   ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();
        telemetry.addLine("This will test:");
        telemetry.addLine("1. Forward 12 inches");
        telemetry.addLine("2. Forward 24 inches");
        telemetry.addLine("3. Forward 48 inches");
        telemetry.addLine("4. Backward 24 inches");
        telemetry.addLine();
        telemetry.addLine("⚠️ MAKE SURE:");
        telemetry.addLine("  • Robot has 8+ feet clearance");
        telemetry.addLine("  • You have tape measure ready");
        telemetry.addLine("  • Starting position is marked");
        telemetry.addLine();
        telemetry.addLine("Press A to start, B to cancel");
        telemetry.update();

        // Wait for confirmation
        while (opModeIsActive() && !gamepad1.a && !gamepad1.b) {
            sleep(50);
        }

        if (gamepad1.b) {
            telemetry.addLine("❌ Test cancelled");
            telemetry.update();
            sleep(1000);
            return;
        }

        // Reset odometry
        driveController.resetOdometry();

        // Store results
        double[] targets = {12.0, 24.0, 48.0, -24.0};
        double[] actuals = new double[4];
        double[] errors = new double[4];
        String[] names = {"12\" forward", "24\" forward", "48\" forward", "24\" backward"};

        // Run tests
        for (int i = 0; i < targets.length; i++) {
            telemetry.clear();
            telemetry.addLine("Test " + (i + 1) + " of " + targets.length);
            telemetry.update();

            testSimpleMovement(targets[i], names[i]);
            actuals[i] = lastTestActualDistance;
            errors[i] = Math.abs(actuals[i] - Math.abs(targets[i]));

            sleep(1000);
        }

        // Display summary
        telemetry.clear();
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║  VERIFICATION COMPLETE        ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();

        double totalError = 0;
        for (int i = 0; i < targets.length; i++) {
            telemetry.addData(names[i], "%.1f\" (err: %.1f\")", actuals[i], errors[i]);
            totalError += errors[i];
        }

        double avgError = totalError / targets.length;
        telemetry.addLine();
        telemetry.addData("Average Error", "%.1f in", avgError);
        telemetry.addLine();

        if (avgError < 2.0) {
            telemetry.addLine("✅ ODOMETRY EXCELLENT");
            telemetry.addLine("Ready for autonomous!");
        } else if (avgError < 4.0) {
            telemetry.addLine("⚠️ ODOMETRY ACCEPTABLE");
            telemetry.addLine("May need minor tuning");
        } else {
            telemetry.addLine("❌ ODOMETRY NEEDS WORK");
            telemetry.addLine("Requires troubleshooting");
        }

        telemetry.addLine();
        telemetry.addLine("NOW: Measure actual distances");
        telemetry.addLine("with tape measure and compare!");

        telemetry.update();
        sleep(5000);
    }

    private void displayStatus() {
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║  ODOMETRY STATUS              ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();

        // Current position
        telemetry.addLine("═══ CURRENT POSITION ═══");
        telemetry.addData("X", "%.2f in", driveController.getX());
        telemetry.addData("Y", "%.2f in", driveController.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(driveController.getHeading()));
        telemetry.addLine();

        // Velocities
        telemetry.addLine("═══ VELOCITIES ═══");
        telemetry.addData("X Velocity", "%.2f in/s", driveController.getVelocityX());
        telemetry.addData("Y Velocity", "%.2f in/s", driveController.getVelocityY());
        telemetry.addLine();

        // Last test results
        if (!lastTestName.equals("None")) {
            telemetry.addLine("═══ LAST TEST ═══");
            telemetry.addData("Test", lastTestName);
            telemetry.addData("Target", "%.1f in", lastTestTargetDistance);
            telemetry.addData("Actual", "%.1f in", lastTestActualDistance);
            double error = Math.abs(lastTestActualDistance - lastTestTargetDistance);
            telemetry.addData("Error", "%.1f in (%.1f%%)",
                    error, (error / lastTestTargetDistance) * 100);
            telemetry.addLine();
        }

        // Pinpoint status
        telemetry.addLine("═══ PINPOINT STATUS ═══");
        telemetry.addData("Status", driveController.getPinpointStatus());
        telemetry.addData("Loop Time", "%d ms", driveController.getPinpointLoopTime());
        telemetry.addData("Frequency", "%.1f Hz", driveController.getPinpointFrequency());
        telemetry.addLine();

        // Encoder raw values
        telemetry.addLine("═══ RAW ENCODERS ═══");
        telemetry.addData("X Encoder", driveController.getXOdoPosition());
        telemetry.addData("Y Encoder", driveController.getYOdoPosition());
        telemetry.addLine();

        // Configuration
        telemetry.addLine("═══ CONFIGURATION ═══");
        telemetry.addData("Pod Type",
                SixWheelDriveController.OdometryParams.USE_4_BAR_PODS ?
                        "4-BAR (Custom)" : "SWINGARM");
        telemetry.addData("X Offset", "%.1f mm",
                SixWheelDriveController.OdometryParams.X_OFFSET_MM);
        telemetry.addData("Y Offset", "%.1f mm",
                SixWheelDriveController.OdometryParams.Y_OFFSET_MM);
        telemetry.addLine();

        telemetry.addLine("Use DPAD to test movements");
        telemetry.addLine("LB: Reset | RB: Verify Test");

        telemetry.update();
    }

    private void displayInstructions() {
        telemetry.addLine("═══ INSTRUCTIONS ═══");
        telemetry.addLine("1. Mark robot start position");
        telemetry.addLine("2. Test forward/backward");
        telemetry.addLine("3. Measure ACTUAL distance");
        telemetry.addLine("4. Compare to odometry reading");
        telemetry.addLine();
        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addLine("DPAD UP: Forward 24\"");
        telemetry.addLine("DPAD DOWN: Backward 24\"");
        telemetry.addLine("X: Forward 48\"");
        telemetry.addLine("A: Forward 12\"");
        telemetry.addLine("B: Backward 48\"");
        telemetry.addLine("Y: Backward 12\"");
        telemetry.addLine("LB: Reset Odometry");
        telemetry.addLine("RB: Run Verification Test");
        telemetry.addLine("START: Emergency Stop");
        telemetry.addLine();
        telemetry.addLine("⚠️ No PID - Simple movement!");
    }
}