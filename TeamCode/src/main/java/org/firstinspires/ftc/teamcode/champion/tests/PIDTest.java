package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;
import org.firstinspires.ftc.teamcode.champion.controller.TurnPIDController;
import org.firstinspires.ftc.teamcode.champion.controller.MovementPIDController;

/**
 * ============================================================================
 * PROVEN PID CONTROLLERS TEST
 * ============================================================================
 *
 * Tests the proven TurnPIDController and MovementPIDController
 * These have been tested and used successfully by many FTC teams
 *
 * CONTROLS:
 * - DPAD UP: Move forward 24 inches
 * - DPAD DOWN: Move backward 24 inches
 * - DPAD LEFT: Turn left 90°
 * - DPAD RIGHT: Turn right 90°
 * - X: Move forward 48 inches
 * - A: Move forward 12 inches
 * - B: Turn 180°
 * - Y: Turn 45°
 * - LEFT BUMPER: Reset odometry
 * - RIGHT BUMPER: Square test
 * - START: Emergency stop
 */
@Config
@TeleOp(name = "Proven PID Test", group = "Test")
public class PIDTest extends LinearOpMode {

    private SixWheelDriveController driveController;
    private TurnPIDController turnPID;
    private MovementPIDController movementPID;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime testTimer = new ElapsedTime();

    // Test parameters
    public static double MOVEMENT_SPEED = 0.6;
    public static double MOVEMENT_TOLERANCE = 2.0;
    public static double TURN_TOLERANCE = 2.0;
    public static double MOVEMENT_TIMEOUT_MS = 5000;
    public static double TURN_TIMEOUT_MS = 3000;
    public static double HEADING_CORRECTION_KP = 0.15;

    // Test tracking
    private String lastTestName = "None";
    private double lastTestDuration = 0;
    private boolean lastTestSuccess = false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize controllers
        driveController = new SixWheelDriveController(this);

        // Initialize IMU separately (Pinpoint doesn't expose getIMU())
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);

        turnPID = new TurnPIDController(imu);
        movementPID = new MovementPIDController();

        displayControls();
        telemetry.addLine();
        telemetry.addLine("✓ Ready! Press START");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driveController.updateOdometry();

            handleControls();
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
            telemetry.addLine(" Odometry reset");
            telemetry.update();
            sleep(300);
            return;
        }

        // Square test
        if (gamepad1.right_bumper) {
            runSquareTest();
            return;
        }

        // Movement tests
        if (gamepad1.dpad_up) {
            testMovement(24.0, "Forward 24\"");
            sleep(300);
        } else if (gamepad1.dpad_down) {
            testMovement(-24.0, "Backward 24\"");
            sleep(300);
        } else if (gamepad1.x) {
            testMovement(48.0, "Forward 48\"");
            sleep(300);
        } else if (gamepad1.a) {
            testMovement(12.0, "Forward 12\"");
            sleep(300);
        }

        // Turn tests
        else if (gamepad1.dpad_left) {
            testTurn(90.0, "Turn Left 90°");
            sleep(300);
        } else if (gamepad1.dpad_right) {
            testTurn(-90.0, "Turn Right 90°");
            sleep(300);
        } else if (gamepad1.b) {
            testTurn(180.0, "Turn 180°");
            sleep(300);
        } else if (gamepad1.y) {
            testTurn(45.0, "Turn 45°");
            sleep(300);
        }
    }

    private void testMovement(double targetDistance, String testName) {
        telemetry.clear();
        telemetry.addLine("Testing: " + testName);
        telemetry.update();

        double startX = driveController.getX();
        double startHeading = driveController.getHeading();
        double direction = Math.signum(targetDistance);

        movementPID.setTarget(Math.abs(targetDistance));
        testTimer.reset();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentDistance = Math.abs(driveController.getX() - startX);

            if (movementPID.isFinished(currentDistance, MOVEMENT_TOLERANCE)) {
                lastTestSuccess = true;
                break;
            }

            if (testTimer.milliseconds() > MOVEMENT_TIMEOUT_MS) {
                lastTestSuccess = false;
                break;
            }

            double pidOutput = movementPID.update(currentDistance);
            double speed = pidOutput * direction;
            speed = Math.max(-MOVEMENT_SPEED, Math.min(MOVEMENT_SPEED, speed));

            // Heading correction
            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);
            double headingCorrection = HEADING_CORRECTION_KP * headingError;

            double leftSpeed = speed - headingCorrection;
            double rightSpeed = speed + headingCorrection;

            leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
            rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

            driveController.tankDriveVelocityNormalized(leftSpeed, rightSpeed);

            telemetry.clear();
            telemetry.addLine("MOVING");
            telemetry.addData("Target", "%.1f in", Math.abs(targetDistance));
            telemetry.addData("Current", "%.1f in", currentDistance);
            telemetry.addData("Error", "%.1f in", movementPID.getError(currentDistance));
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.update();

            sleep(10);
        }

        driveController.stopDrive();

        lastTestName = testName;
        lastTestDuration = testTimer.seconds();

        displayTestResult();
    }

    private void testTurn(double angleDegrees, String testName) {
        telemetry.clear();
        telemetry.addLine("Testing: " + testName);
        telemetry.update();

        double startHeading = turnPID.getCurrentHeading();
        double targetHeading = startHeading + angleDegrees;

        turnPID.setTarget(targetHeading);
        testTimer.reset();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opModeIsActive()) {
            driveController.updateOdometry();

            if (turnPID.isFinished(TURN_TOLERANCE)) {
                lastTestSuccess = true;
                break;
            }

            if (testTimer.milliseconds() > TURN_TIMEOUT_MS) {
                lastTestSuccess = false;
                break;
            }

            double power = turnPID.update();
            driveController.tankDrive(-power, power);

            telemetry.clear();
            telemetry.addLine("TURNING");
            telemetry.addData("Target", "%.1f°", targetHeading);
            telemetry.addData("Current", "%.1f°", turnPID.getCurrentHeading());
            telemetry.addData("Error", "%.1f°", turnPID.getError());
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();

            sleep(10);
        }

        driveController.stopDrive();

        lastTestName = testName;
        lastTestDuration = testTimer.seconds();

        displayTestResult();
    }

    /**
     * Run square test
     */
    private void runSquareTest() {
        telemetry.clear();
        telemetry.addLine("Running Square Test...");
        telemetry.update();

        double startX = driveController.getX();
        double startY = driveController.getY();

        for (int i = 0; i < 4; i++) {
            testMovement(24.0, "Side " + (i + 1));
            sleep(300);
            testTurn(-90.0, "Corner " + (i + 1));
            sleep(300);
        }

        double driftX = Math.abs(driveController.getX() - startX);
        double driftY = Math.abs(driveController.getY() - startY);
        double totalDrift = Math.sqrt(driftX * driftX + driftY * driftY);

        telemetry.clear();
        telemetry.addLine("═══ SQUARE TEST COMPLETE ═══");
        telemetry.addData("Total Drift", "%.2f in", totalDrift);

        if (totalDrift < 4.0) {
            telemetry.addLine("✅ EXCELLENT");
        } else if (totalDrift < 8.0) {
            telemetry.addLine("⚠️ ACCEPTABLE");
        } else {
            telemetry.addLine("❌ NEEDS WORK");
        }

        telemetry.update();
        sleep(3000);
    }

    private void displayTestResult() {
        telemetry.clear();
        telemetry.addLine("═══ TEST COMPLETE ═══");
        telemetry.addData("Test", lastTestName);
        telemetry.addData("Duration", "%.2f sec", lastTestDuration);
        telemetry.addData("Result", lastTestSuccess ? "✅ SUCCESS" : "⚠️ TIMEOUT");
        telemetry.update();
        sleep(1500);
    }

    private void displayStatus() {
        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║  PROVEN PID TEST              ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();

        telemetry.addLine("═══ POSITION ═══");
        telemetry.addData("X", "%.2f in", driveController.getX());
        telemetry.addData("Y", "%.2f in", driveController.getY());
        telemetry.addData("Heading", "%.1f°", turnPID.getCurrentHeading());
        telemetry.addLine();

        telemetry.addLine("═══ PID VALUES ═══");
        telemetry.addData("Turn kP", "%.4f", TurnPIDController.kP);
        telemetry.addData("Turn kI", "%.4f", TurnPIDController.kI);
        telemetry.addData("Turn kD", "%.4f", TurnPIDController.kD);
        telemetry.addLine();
        telemetry.addData("Move kP", "%.4f", MovementPIDController.kP);
        telemetry.addData("Move kI", "%.4f", MovementPIDController.kI);
        telemetry.addData("Move kD", "%.4f", MovementPIDController.kD);
        telemetry.addLine();

        if (!lastTestName.equals("None")) {
            telemetry.addLine("═══ LAST TEST ═══");
            telemetry.addData("Test", lastTestName);
            telemetry.addData("Duration", "%.2f sec", lastTestDuration);
        }

        telemetry.addLine();
        telemetry.addLine("Use DPAD for tests");

        telemetry.update();
    }

    private void displayControls() {
        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addLine("DPAD: Move/Turn");
        telemetry.addLine("X/A: Long/Short Forward");
        telemetry.addLine("B: Turn 180°");
        telemetry.addLine("Y: Turn 45°");
        telemetry.addLine("LB: Reset");
        telemetry.addLine("RB: Square Test");
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}