package org.firstinspires.ftc.teamcode.champion.tests;

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
public class NewPIDTest extends LinearOpMode {
    SixWheelDriveController driveController;

    // Test parameters
    public static double TEST_DISTANCE = 30.0;  // inches
    public static double TEST_ANGLE = 90.0;      // degrees
    public static double MOVEMENT_SPEED = 0.6;

    // ========== MOVEMENT PID PARAMETERS ==========
    // These match SimpleBasicAuton.MovementPID and NewAutonController.MovementPID
    @Config
    public static class MovementPID {
        public static double kP = 0.034;
        public static double kI = 0.0;
        public static double kD = 0.0093;
        public static double MIN_SPEED = 0.1;
        public static double MAX_SPEED = 0.8;
        public static double TOLERANCE = 2.0;
        public static double TIMEOUT_MS = 5000;
        public static double HEADING_CORRECTION_KP = 0.15;
        public static double DECEL_DISTANCE = 16.0;
    }

    // ========== TURN PID PARAMETERS ==========
    // These match SimpleBasicAuton.TurnPID and NewAutonController.TurnPID
    @Config
    public static class TurnPID {
        public static double kP = 0.65;
        public static double kI = 0.0;
        public static double kD = 0.03;
        public static double MIN_POWER = 0.20;
        public static double MAX_POWER = 0.65;
        public static double TOLERANCE_DEG = 2.0;
        public static double TIMEOUT_MS = 3000;
    }

    // Button state tracking
    private boolean lastAState = false;
    private boolean lastBState = false;
    private boolean lastXState = false;
    private boolean lastYState = false;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize
        driveController = new SixWheelDriveController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== PID TUNING TEST ===");
        telemetry.addLine("Uses SimplyPID library (same as auton)");
        telemetry.addLine();
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
                driveDistancePID(TEST_DISTANCE, MOVEMENT_SPEED);
                displayResults();
            }

            // B Button - Move Backward
            if (currentBState && !lastBState) {
                telemetry.addLine(">>> MOVING BACKWARD 30 INCHES <<<");
                telemetry.update();
                driveDistancePID(-TEST_DISTANCE, MOVEMENT_SPEED);
                displayResults();
            }

            // X Button - Turn Left 90°
            if (currentXState && !lastXState) {
                telemetry.addLine(">>> TURNING LEFT 90° <<<");
                telemetry.update();
                double currentHeading = Math.toDegrees(driveController.getHeading());
                turnToHeadingPID(currentHeading + TEST_ANGLE);
                displayResults();
            }

            // Y Button - Turn Right 90°
            if (currentYState && !lastYState) {
                telemetry.addLine(">>> TURNING RIGHT 90° <<<");
                telemetry.update();
                double currentHeading = Math.toDegrees(driveController.getHeading());
                turnToHeadingPID(currentHeading - TEST_ANGLE);
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
            telemetry.addData("kP", MovementPID.kP);
            telemetry.addData("kI", MovementPID.kI);
            telemetry.addData("kD", MovementPID.kD);
            telemetry.addData("Decel Distance", MovementPID.DECEL_DISTANCE);
            telemetry.addLine("--- Turn PID ---");
            telemetry.addData("kP", TurnPID.kP);
            telemetry.addData("kI", TurnPID.kI);
            telemetry.addData("kD", TurnPID.kD);
            telemetry.update();

            sleep(50);
        }
    }

    private void driveDistancePID(double distanceInches, double maxSpeed) {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double startHeading = driveController.getHeading();
        double targetDistance = Math.abs(distanceInches);
        double direction = Math.signum(distanceInches);

        // Create PID controller using SimplyPID library (same as auton)
        fr.charleslabs.simplypid.SimplyPID movePID = new fr.charleslabs.simplypid.SimplyPID(
                0.0,
                MovementPID.kP,
                MovementPID.kI,
                MovementPID.kD
        );
        movePID.setOuputLimits(-MovementPID.MAX_SPEED, MovementPID.MAX_SPEED);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        double maxDistanceReached = 0.0;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            // Calculate distance traveled
            double currentDistance = Math.abs(driveController.getX() - startX);
            maxDistanceReached = Math.max(maxDistanceReached, currentDistance);
            double error = targetDistance - currentDistance;

            // Heading correction to drive straight
            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);
            double headingCorrection = MovementPID.HEADING_CORRECTION_KP * headingError;

            // Check if we've reached the target
            if (Math.abs(error) < MovementPID.TOLERANCE) {
                break;
            }

            // Timeout check
            if (timer.milliseconds() > MovementPID.TIMEOUT_MS) {
                break;
            }

            // Get PID output
            double pidOutput = movePID.getOutput(timer.seconds(), error);
            double speed = pidOutput;

            // Deceleration as we approach target
            double effectiveMaxSpeed = maxSpeed;
            if (Math.abs(error) < MovementPID.DECEL_DISTANCE) {
                double decelFactor = Math.abs(error) / MovementPID.DECEL_DISTANCE;
                decelFactor = decelFactor * decelFactor;
                effectiveMaxSpeed = MovementPID.MIN_SPEED +
                        (maxSpeed - MovementPID.MIN_SPEED) * decelFactor;
            }

            // Ensure minimum speed
            if (Math.abs(speed) < MovementPID.MIN_SPEED && Math.abs(error) > MovementPID.TOLERANCE * 2) {
                speed = Math.signum(speed) * MovementPID.MIN_SPEED;
            }

            // Clamp speed
            speed = Math.max(-effectiveMaxSpeed, Math.min(effectiveMaxSpeed, speed));
            speed *= direction;

            // Apply heading correction
            double leftSpeed = speed - headingCorrection;
            double rightSpeed = speed + headingCorrection;

            leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
            rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

            driveController.tankDriveVelocityNormalized(leftSpeed, rightSpeed);

            // Telemetry
            telemetry.addLine("=== MOVING ===");
            telemetry.addData("Target", "%.1f in", Math.abs(distanceInches));
            telemetry.addData("Current", "%.1f in", currentDistance);
            telemetry.addData("Error", "%.1f in", error);
            telemetry.addData("Speed", "%.3f", speed);
            telemetry.addData("Effective Max", "%.3f", effectiveMaxSpeed);
            telemetry.addData("Heading Error", "%.2f°", Math.toDegrees(headingError));
            telemetry.addData("Time", "%.2f s", timer.seconds());
            telemetry.update();

            sleep(10);
        }

        driveController.stopDrive();
        sleep(100);

        // Final report
        driveController.updateOdometry();
        double finalDistance = Math.abs(driveController.getX() - startX);
        double finalError = targetDistance - finalDistance;

        telemetry.addLine();
        telemetry.addLine("=== MOVEMENT COMPLETE ===");
        telemetry.addData("Target Distance", "%.2f in", targetDistance);
        telemetry.addData("Final Distance", "%.2f in", finalDistance);
        telemetry.addData("Final Error", "%.2f in", finalError);
        telemetry.addData("Max Distance Reached", "%.2f in", maxDistanceReached);
        telemetry.addData("Total Time", "%.2f s", timer.seconds());
        telemetry.update();
    }

    private void turnToHeadingPID(double targetDegrees) {
        double startHeading = Math.toDegrees(driveController.getHeading());
        double targetRad = Math.toRadians(targetDegrees);
        double toleranceRad = Math.toRadians(TurnPID.TOLERANCE_DEG);

        // Create PID controller using SimplyPID library (same as auton)
        fr.charleslabs.simplypid.SimplyPID turnPID = new fr.charleslabs.simplypid.SimplyPID(
                0.0,
                TurnPID.kP,
                TurnPID.kI,
                TurnPID.kD
        );
        turnPID.setOuputLimits(-TurnPID.MAX_POWER, TurnPID.MAX_POWER);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        double maxErrorSeen = 0.0;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(targetRad - currentHeading);
            maxErrorSeen = Math.max(maxErrorSeen, Math.abs(Math.toDegrees(headingError)));

            // Check if we've reached the target
            if (Math.abs(headingError) < toleranceRad) {
                break;
            }

            // Timeout check
            if (timer.milliseconds() > TurnPID.TIMEOUT_MS) {
                break;
            }

            // Get PID output
            double pidOutput = turnPID.getOutput(timer.seconds(), headingError);
            double power = pidOutput;

            // Ensure minimum power
            if (Math.abs(power) < TurnPID.MIN_POWER && Math.abs(headingError) > toleranceRad) {
                power = Math.signum(power) * TurnPID.MIN_POWER;
            }

            // Apply turn power (left negative, right positive for right turn)
            driveController.tankDrive(-power, power);

            // Telemetry
            telemetry.addLine("=== TURNING ===");
            telemetry.addData("Target", "%.1f°", targetDegrees);
            telemetry.addData("Current", "%.1f°", Math.toDegrees(currentHeading));
            telemetry.addData("Error", "%.1f°", Math.toDegrees(headingError));
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Time", "%.2f s", timer.seconds());
            telemetry.update();

            sleep(10);
        }

        driveController.stopDrive();
        sleep(100);

        // Final report
        driveController.updateOdometry();
        double finalHeading = Math.toDegrees(driveController.getHeading());
        double finalError = targetDegrees - finalHeading;

        // Normalize final error
        while (finalError > 180) finalError -= 360;
        while (finalError < -180) finalError += 360;

        telemetry.addLine();
        telemetry.addLine("=== TURN COMPLETE ===");
        telemetry.addData("Starting Heading", "%.2f°", startHeading);
        telemetry.addData("Target Heading", "%.2f°", targetDegrees);
        telemetry.addData("Final Heading", "%.2f°", finalHeading);
        telemetry.addData("Final Error", "%.2f°", finalError);
        telemetry.addData("Max Error Seen", "%.2f°", maxErrorSeen);
        telemetry.addData("Heading Change", "%.2f°", finalHeading - startHeading);
        telemetry.addData("Total Time", "%.2f s", timer.seconds());
        telemetry.update();
    }

    /**
     * Display results after a test
     */
    private void displayResults() {
        telemetry.addLine();
        telemetry.addLine("=== TEST COMPLETE ===");
        telemetry.addLine("Press another button to test again");
        telemetry.update();
        sleep(1500);
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