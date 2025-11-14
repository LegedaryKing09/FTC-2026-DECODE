package org.firstinspires.ftc.teamcode.champion.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

import java.util.Locale;

@Config
@TeleOp(name = "PID Tuning Test", group = "Testing")
public class PIDTuningTest extends LinearOpMode {

    SixWheelDriveController driveController;

    // Test parameters
    public static double TEST_DISTANCE = 30.0;  // inches
    public static double TEST_ANGLE = 90.0;      // degrees
    public static double MOVEMENT_SPEED = 0.5;
    public static double HEADING_THRESHOLD_DEG = 2.0;

    // ========== MOVEMENT PID COEFFICIENTS ==========
    @Config
    public static class TestingMovementPID {
        public static double kP = 0.06;  // Proportional gain for distance error
        public static double kI = 0.0;   // Integral gain
        public static double kD = 0.017;  // Derivative gain
        public static double MIN_SPEED = 0.05;  // Minimum speed to overcome friction
        public static double MAX_SPEED = 0.8;   // Maximum speed cap
    }

    // ========== TURN PID COEFFICIENTS ==========
    @Config
    public static class TestingTurnPID {
        public static double kP = 0.65;   // Proportional gain for heading error
        public static double kI = 0.0;   // Integral gain
        public static double kD = 0.04;  // Derivative gain
        public static double MIN_POWER = 0.15;  // Minimum turn power
        public static double MAX_POWER = 0.65;   // Maximum turn power
    }

    // PID state variables for movement
    private double movementIntegral = 0.0;
    private double movementLastError = 0.0;

    // PID state variables for turning
    private double turnIntegral = 0.0;
    private double turnLastError = 0.0;

    // Button state tracking
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;

    @Override
    public void runOpMode() {
        // Initialize
        driveController = new SixWheelDriveController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== PID TUNING TEST ===");
        telemetry.addLine("A - Move Forward 30in");
        telemetry.addLine("B - Move Backward 30in");
        telemetry.addLine("X - Turn Left 90°");
        telemetry.addLine("Y - Turn Right 90°");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // Main loop
        while (opModeIsActive()) {
            driveController.updateOdometry();

            // Check for button presses (with edge detection to prevent repeated triggers)
            boolean currentAState = gamepad1.a;
            boolean currentBState = gamepad1.b;
            boolean currentXState = gamepad1.x;
            boolean currentYState = gamepad1.y;

            // A Button - Move Forward
            if (currentAState && !lastAState) {
                telemetry.addLine(">>> MOVING FORWARD 30 INCHES <<<");
                telemetry.update();
                moveRobot(TEST_DISTANCE, MOVEMENT_SPEED);
                displayResults();
            }

            // B Button - Move Backward
            if (currentBState && !lastBState) {
                telemetry.addLine(">>> MOVING BACKWARD 30 INCHES <<<");
                telemetry.update();
                moveRobot(-TEST_DISTANCE, MOVEMENT_SPEED);
                displayResults();
            }

            // X Button - Turn Left 90°
            if (currentXState && !lastXState) {
                telemetry.addLine(">>> TURNING LEFT 90° <<<");
                telemetry.update();
                double currentHeading = Math.toDegrees(driveController.getHeading());
                turnToHeading(currentHeading + TEST_ANGLE);
                displayResults();
            }

            // Y Button - Turn Right 90°
            if (currentYState && !lastYState) {
                telemetry.addLine(">>> TURNING RIGHT 90° <<<");
                telemetry.update();
                double currentHeading = Math.toDegrees(driveController.getHeading());
                turnToHeading(currentHeading - TEST_ANGLE);
                displayResults();
            }

            // Update button states
            lastAState = currentAState;
            lastBState = currentBState;
            lastXState = currentXState;
            lastYState = currentYState;

            // Display current status
            telemetry.addLine("=== READY FOR TESTING ===");
            telemetry.addLine("A - Move Forward 30in");
            telemetry.addLine("B - Move Backward 30in");
            telemetry.addLine("X - Turn Left 90°");
            telemetry.addLine("Y - Turn Right 90°");
            telemetry.addLine();
            telemetry.addData("Current X", String.format(Locale.US, "%.2f in", driveController.getX()));
            telemetry.addData("Current Y", String.format(Locale.US, "%.2f in", driveController.getY()));
            telemetry.addData("Current Heading", String.format(Locale.US, "%.2f°", driveController.getHeadingDegrees()));
            telemetry.addLine();
            telemetry.addLine("--- Movement PID ---");
            telemetry.addData("kP", TestingMovementPID.kP);
            telemetry.addData("kI", TestingMovementPID.kI);
            telemetry.addData("kD", TestingMovementPID.kD);
            telemetry.addLine("--- Turn PID ---");
            telemetry.addData("kP", TestingTurnPID.kP);
            telemetry.addData("kI", TestingTurnPID.kI);
            telemetry.addData("kD", TestingTurnPID.kD);
            telemetry.update();

            sleep(50);
        }
    }

    /**
     * Movement with PID control for smooth acceleration and deceleration
     */
    private void moveRobot(double distanceInches, double maxSpeed) {
        // Reset PID state
        movementIntegral = 0.0;
        movementLastError = 0.0;

        driveController.updateOdometry();
        double startX = driveController.getX();
        double startY = driveController.getY();
        double targetDistance = Math.abs(distanceInches);
        double direction = Math.signum(distanceInches);

        ElapsedTime moveTimer = new ElapsedTime();
        ElapsedTime pidTimer = new ElapsedTime();

        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentDistance = Math.abs(driveController.getX() - startX);
            double distanceError = targetDistance - currentDistance;

            // Calculate PID output
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Proportional term
            double pTerm = TestingMovementPID.kP * distanceError;

            // Integral term (with anti-windup)
            movementIntegral += distanceError * dt;
            movementIntegral = Math.max(-10, Math.min(10, movementIntegral)); // Clamp integral
            double iTerm = TestingMovementPID.kI * movementIntegral;

            // Derivative term
            double dTerm = 0.0;
            if (dt > 0) {
                dTerm = TestingMovementPID.kD * (distanceError - movementLastError) / dt;
            }
            movementLastError = distanceError;

            // Calculate speed with PID
            double speed = pTerm + iTerm + dTerm;

            // Apply speed limits
            speed = Math.max(TestingMovementPID.MIN_SPEED, Math.min(maxSpeed, Math.abs(speed)));
            speed = Math.min(speed, TestingMovementPID.MAX_SPEED);

            // Apply direction
            speed *= direction;

            driveController.tankDriveVelocityNormalized(speed, speed);

            // Telemetry updates
            if (moveTimer.milliseconds() % 50 < 10) {
                telemetry.addLine("=== MOVING ===");
                telemetry.addData("Distance Error", String.format(Locale.US, "%.2f in", distanceError));
                telemetry.addData("Current Distance", String.format(Locale.US, "%.2f in", currentDistance));
                telemetry.addData("Target Distance", String.format(Locale.US, "%.2f in", targetDistance));
                telemetry.addData("Speed", String.format(Locale.US, "%.3f", speed));
                telemetry.addData("P Term", String.format(Locale.US, "%.3f", pTerm));
                telemetry.addData("I Term", String.format(Locale.US, "%.3f", iTerm));
                telemetry.addData("D Term", String.format(Locale.US, "%.3f", dTerm));
                telemetry.addData("Time Elapsed", String.format(Locale.US, "%.2f s", moveTimer.seconds()));
                telemetry.update();
            }

            // Exit condition - within 0.5 inches or past target
            if (distanceError < 0.5 || currentDistance >= targetDistance) {
                break;
            }
            sleep(5);
        }
        driveController.stopDrive();

        // Reset integral for next movement
        movementIntegral = 0.0;

        telemetry.addLine("=== MOVEMENT COMPLETE ===");
        telemetry.addData("Final Distance", String.format(Locale.US, "%.2f in",
                Math.abs(driveController.getX() - startX)));
        telemetry.addData("Total Time", String.format(Locale.US, "%.2f s", moveTimer.seconds()));
        telemetry.update();
        sleep(500);
    }

    /**
     * Turn with PID control for smooth rotation
     */
    private void turnToHeading(double targetDegrees) {
        // Reset PID state
        turnIntegral = 0.0;
        turnLastError = 0.0;

        double startHeading = driveController.getHeadingDegrees();
        double targetRad = Math.toRadians(targetDegrees);
        double thresholdRad = Math.toRadians(HEADING_THRESHOLD_DEG);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime pidTimer = new ElapsedTime();

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double headingError = normalizeAngle(targetRad - driveController.getHeading());

            // Exit if within threshold
            if (Math.abs(headingError) < thresholdRad) break;

            // Calculate PID output
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Proportional term
            double pTerm = TestingTurnPID.kP * headingError;

            // Integral term (with anti-windup)
            turnIntegral += headingError * dt;
            turnIntegral = Math.max(-5, Math.min(5, turnIntegral)); // Clamp integral
            double iTerm = TestingTurnPID.kI * turnIntegral;

            // Derivative term
            double dTerm = 0.0;
            if (dt > 0) {
                dTerm = TestingTurnPID.kD * (headingError - turnLastError) / dt;
            }
            turnLastError = headingError;

            // Calculate turn power with PID
            double power = pTerm + iTerm + dTerm;

            // Apply power limits and minimum power
            double absPower = Math.abs(power);
            if (absPower < TestingTurnPID.MIN_POWER) {
                power = Math.signum(power) * TestingTurnPID.MIN_POWER;
            } else if (absPower > TestingTurnPID.MAX_POWER) {
                power = Math.signum(power) * TestingTurnPID.MAX_POWER;
            }

            driveController.tankDrive(-power, power);

            // Telemetry during turn
            if (turnTimer.milliseconds() % 50 < 10) {
                telemetry.addLine("=== TURNING ===");
                telemetry.addData("Heading Error", String.format(Locale.US, "%.2f°", Math.toDegrees(headingError)));
                telemetry.addData("Current Heading", String.format(Locale.US, "%.2f°", driveController.getHeadingDegrees()));
                telemetry.addData("Target Heading", String.format(Locale.US, "%.2f°", targetDegrees));
                telemetry.addData("Turn Power", String.format(Locale.US, "%.3f", power));
                telemetry.addData("P Term", String.format(Locale.US, "%.3f", pTerm));
                telemetry.addData("I Term", String.format(Locale.US, "%.3f", iTerm));
                telemetry.addData("D Term", String.format(Locale.US, "%.3f", dTerm));
                telemetry.addData("Time Elapsed", String.format(Locale.US, "%.2f s", turnTimer.seconds()));
                telemetry.update();
            }

            sleep(5);
        }
        driveController.stopDrive();

        // Reset integral for next turn
        turnIntegral = 0.0;

        telemetry.addLine("=== TURN COMPLETE ===");
        telemetry.addData("Starting Heading", String.format(Locale.US, "%.2f°", startHeading));
        telemetry.addData("Final Heading", String.format(Locale.US, "%.2f°", driveController.getHeadingDegrees()));
        telemetry.addData("Heading Change", String.format(Locale.US, "%.2f°",
                driveController.getHeadingDegrees() - startHeading));
        telemetry.addData("Total Time", String.format(Locale.US, "%.2f s", turnTimer.seconds()));
        telemetry.update();
        sleep(500);
    }

    /**
     * Display results after a test
     */
    private void displayResults() {
        telemetry.addLine();
        telemetry.addLine("=== TEST COMPLETE ===");
        telemetry.addLine("Press another button to test again");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Normalize angle to [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}