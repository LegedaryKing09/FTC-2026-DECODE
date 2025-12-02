package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

import fr.charleslabs.simplypid.SimplyPID;

@Config
@Autonomous(name = "PID Tuner", group = "Testing")
public class PIDTuningTest extends LinearOpMode {

    public enum TestMode {
        MOVEMENT,
        TURN
    }

    public static TestMode TEST_MODE = TestMode.MOVEMENT;

    // ========== MOVEMENT TEST PARAMETERS ==========
    @Config
    public static class MovementTest {
        public static double TARGET_DISTANCE = 48.0;
        public static double kP = 0.0333;
        public static double kI = 0.0;
        public static double kD = 0.01;
        public static double MIN_SPEED = 0.1;
        public static double MAX_SPEED = 0.7;
        public static double TOLERANCE = 2.0;
        public static double TIMEOUT_MS = 5000;

        // HEADING CORRECTION PID
        public static double HEADING_kP = 0.25;
        public static double HEADING_kI = 0.0;
        public static double HEADING_kD = 0.15;  // Increased for damping
        public static double MAX_HEADING_CORRECTION = 0.3;

        public static double DECEL_DISTANCE = 15.0;

        // STABLE SETTLING PARAMETERS - KEY FIX!
        public static int STABLE_COUNTS_REQUIRED = 15;  // Must be stable for 15 loops (150ms)
        public static double STABLE_SPEED_THRESHOLD = 0.05;  // Speed must be < 0.05
    }

    // ========== TURN TEST PARAMETERS ==========
    @Config
    public static class TurnTest {
        public static double TARGET_ANGLE = 90.0;
        public static double kP = 0.65;
        public static double kI = 0.0;
        public static double kD = 0.03;
        public static double MIN_POWER = 0.15;
        public static double MAX_POWER = 0.65;
        public static double TOLERANCE_DEG = 2.0;
        public static double TIMEOUT_MS = 3000;

        // STABLE SETTLING FOR TURNS
        public static int STABLE_COUNTS_REQUIRED = 15;
    }

    public static boolean REPEAT_TEST = false;
    public static int REPEAT_COUNT = 3;

    private SixWheelDriveController driveController;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        driveController = new SixWheelDriveController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== PID TUNER ===");
        telemetry.addData("Test Mode", TEST_MODE);
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        int iterations = REPEAT_TEST ? REPEAT_COUNT : 1;

        for (int i = 0; i < iterations && opModeIsActive(); i++) {
            if (TEST_MODE == TestMode.MOVEMENT) {
                testMovement();
            } else {
                testTurn();
            }

            if (REPEAT_TEST && i < iterations - 1) {
                sleep(2000);
            }
        }

        telemetry.addLine("=== TEST COMPLETE ===");
        telemetry.update();
    }

    private void testMovement() {
        driveController.updateOdometry();
        double startX = driveController.getX();
        double startHeading = driveController.getHeading();
        double targetDistance = Math.abs(MovementTest.TARGET_DISTANCE);
        double direction = Math.signum(MovementTest.TARGET_DISTANCE);

        // Movement PID
        SimplyPID movePID = new SimplyPID(
                0.0,
                MovementTest.kP,
                MovementTest.kI,
                MovementTest.kD
        );
        movePID.setOuputLimits(-MovementTest.MAX_SPEED, MovementTest.MAX_SPEED);

        // HEADING CORRECTION PID
        SimplyPID headingPID = new SimplyPID(
                0.0,
                MovementTest.HEADING_kP,
                MovementTest.HEADING_kI,
                MovementTest.HEADING_kD
        );
        headingPID.setOuputLimits(-MovementTest.MAX_HEADING_CORRECTION, MovementTest.MAX_HEADING_CORRECTION);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        double maxError = 0;
        double maxHeadingError = 0;
        double settlingTime = 0;
        boolean settled = false;

        // STABLE SETTLING TRACKING
        int stableCount = 0;
        double previousSpeed = 0;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentDistance = Math.abs(driveController.getX() - startX);
            double error = targetDistance - currentDistance;

            // Heading error calculation
            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);

            if (Math.abs(headingError) > Math.abs(maxHeadingError)) {
                maxHeadingError = headingError;
            }

            // Use PID for heading correction
            double headingCorrection = headingPID.getOutput(timer.seconds(), -headingError);

            if (Math.abs(error) > Math.abs(maxError)) {
                maxError = error;
            }

            if (!settled && Math.abs(error) < MovementTest.TOLERANCE) {
                settlingTime = timer.seconds();
                settled = true;
            }

            // ========== STABLE EXIT CONDITION - KEY FIX! ==========
            // Check if robot is within tolerance AND stable (not moving much)
            boolean withinTolerance = Math.abs(error) < MovementTest.TOLERANCE;

            // Get base speed from PID
            double pidOutput = movePID.getOutput(timer.seconds(), -error);
            double speed = pidOutput;

            // DECELERATION ZONE
            double effectiveMaxSpeed = MovementTest.MAX_SPEED;
            if (Math.abs(error) < MovementTest.DECEL_DISTANCE) {
                double decelFactor = Math.abs(error) / MovementTest.DECEL_DISTANCE;
                decelFactor = decelFactor * decelFactor;
                effectiveMaxSpeed = MovementTest.MIN_SPEED +
                        (MovementTest.MAX_SPEED - MovementTest.MIN_SPEED) * decelFactor;
            }

            // Apply MIN_SPEED only when far from target
            if (Math.abs(speed) < MovementTest.MIN_SPEED && Math.abs(error) > MovementTest.TOLERANCE * 2) {
                speed = Math.signum(speed) * MovementTest.MIN_SPEED;
            }

            // Clamp to effective max speed
            speed = Math.max(-effectiveMaxSpeed, Math.min(effectiveMaxSpeed, speed));

            // Check if speed is stable (not changing much)
            boolean speedStable = Math.abs(speed) < MovementTest.STABLE_SPEED_THRESHOLD;

            // Count consecutive stable readings
            if (withinTolerance && speedStable) {
                stableCount++;
            } else {
                stableCount = 0;  // Reset if not stable
            }

            // Exit ONLY when stable for required number of loops
            if (stableCount >= MovementTest.STABLE_COUNTS_REQUIRED) {
                break;  // Robot is stable at target!
            }

            if (timer.milliseconds() > MovementTest.TIMEOUT_MS) {
                break;
            }

            speed *= direction;

            // Apply heading correction
            double leftSpeed = speed - headingCorrection;
            double rightSpeed = speed + headingCorrection;

            leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
            rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

            driveController.tankDriveVelocityNormalized(leftSpeed, rightSpeed);

            // Enhanced telemetry for tuning
            telemetry.addData("Distance Error", "%.2f in", error);
            telemetry.addData("Heading Error", "%.2f°", Math.toDegrees(headingError));
            telemetry.addData("Speed", "%.3f", speed);
            telemetry.addData("Stable Count", "%d / %d", stableCount, MovementTest.STABLE_COUNTS_REQUIRED);
            telemetry.addData("Status", stableCount > 0 ? "SETTLING..." : "MOVING");
            telemetry.update();

            previousSpeed = speed;
            sleep(10);
        }

        driveController.stopDrive();

        // Brief settling period
        sleep(100);

        // Final results
        driveController.updateOdometry();
        double finalDistance = Math.abs(driveController.getX() - startX);
        double finalError = targetDistance - finalDistance;
        double finalHeadingError = normalizeAngle(startHeading - driveController.getHeading());

        telemetry.addLine("=== RESULTS ===");
        telemetry.addData("Target", "%.1f in", targetDistance);
        telemetry.addData("Actual", "%.1f in", finalDistance);
        telemetry.addData("Distance Error", "%.2f in", finalError);
        telemetry.addData("Time", "%.2fs", timer.seconds());
        if (settled) {
            telemetry.addData("Settling", "%.2fs", settlingTime);
        }
        telemetry.addData("Accuracy", "%.1f%%", 100 * (1 - Math.abs(finalError) / targetDistance));
        telemetry.addLine();
        telemetry.addData("Final Heading Error", "%.2f°", Math.toDegrees(finalHeadingError));
        telemetry.addData("Max Heading Error", "%.2f°", Math.toDegrees(maxHeadingError));
        telemetry.update();
    }

    private void testTurn() {
        double targetRad = Math.toRadians(TurnTest.TARGET_ANGLE);
        double toleranceRad = Math.toRadians(TurnTest.TOLERANCE_DEG);

        SimplyPID turnPID = new SimplyPID(
                0.0,
                TurnTest.kP,
                TurnTest.kI,
                TurnTest.kD
        );
        turnPID.setOuputLimits(-TurnTest.MAX_POWER, TurnTest.MAX_POWER);

        driveController.updateOdometry();
        double startHeading = driveController.getHeading();

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        double maxError = 0;
        double settlingTime = 0;
        boolean settled = false;

        // STABLE SETTLING TRACKING
        int stableCount = 0;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(targetRad - currentHeading);

            if (Math.abs(headingError) > Math.abs(maxError)) {
                maxError = headingError;
            }

            if (!settled && Math.abs(headingError) < toleranceRad) {
                settlingTime = timer.seconds();
                settled = true;
            }

            // STABLE EXIT CONDITION
            boolean withinTolerance = Math.abs(headingError) < toleranceRad;

            if (withinTolerance) {
                stableCount++;
            } else {
                stableCount = 0;
            }

            // Exit when stable for required count
            if (stableCount >= TurnTest.STABLE_COUNTS_REQUIRED) {
                break;
            }

            if (timer.milliseconds() > TurnTest.TIMEOUT_MS) {
                break;
            }

            double pidOutput = turnPID.getOutput(timer.seconds(), -headingError);

            double power = pidOutput;
            if (Math.abs(power) < TurnTest.MIN_POWER && Math.abs(headingError) > toleranceRad) {
                power = Math.signum(power) * TurnTest.MIN_POWER;
            }

            driveController.tankDrive(-power, power);

            telemetry.addData("Heading Error", "%.2f°", Math.toDegrees(headingError));
            telemetry.addData("Power", "%.3f", power);
            telemetry.addData("Stable Count", "%d / %d", stableCount, TurnTest.STABLE_COUNTS_REQUIRED);
            telemetry.update();

            sleep(10);
        }

        driveController.stopDrive();

        // Brief settling period
        sleep(100);

        driveController.updateOdometry();
        double finalHeading = driveController.getHeading();
        double finalError = normalizeAngle(targetRad - finalHeading);

        telemetry.addLine("=== RESULTS ===");
        telemetry.addData("Target", "%.1f°", TurnTest.TARGET_ANGLE);
        telemetry.addData("Actual", "%.1f°", Math.toDegrees(finalHeading));
        telemetry.addData("Error", "%.2f°", Math.toDegrees(finalError));
        telemetry.addData("Time", "%.2fs", timer.seconds());
        if (settled) {
            telemetry.addData("Settling", "%.2fs", settlingTime);
        }
        telemetry.addData("Accuracy", "%.1f%%",
                100 * (1 - Math.abs(Math.toDegrees(finalError)) / Math.abs(TurnTest.TARGET_ANGLE)));
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}