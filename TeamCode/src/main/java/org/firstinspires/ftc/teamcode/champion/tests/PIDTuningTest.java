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
        public static double kP = 0.03;
        public static double kI = 0.0;
        public static double kD = 0.007;
        public static double MIN_SPEED = 0.1;
        public static double MAX_SPEED = 0.8;
        public static double TOLERANCE = 1.0;
        public static double TIMEOUT_MS = 5000;
        public static double HEADING_CORRECTION_KP = 0.1;
        public static double DECEL_DISTANCE = 9.2;
    }

    // ========== TURN TEST PARAMETERS ==========
    @Config
    public static class TurnTest {
        public static double TARGET_ANGLE = 90.0;
        public static double kP = 0.65;
        public static double kI = 0.0;
        public static double kD = 0.04;
        public static double MIN_POWER = 0.15;
        public static double MAX_POWER = 0.65;
        public static double TOLERANCE_DEG = 2.0;
        public static double TIMEOUT_MS = 3000;
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

        SimplyPID movePID = new SimplyPID(
                0.0,
                MovementTest.kP,
                MovementTest.kI,
                MovementTest.kD
        );
        movePID.setOuputLimits(-MovementTest.MAX_SPEED, MovementTest.MAX_SPEED);

        timer.reset();
        driveController.setDriveMode(SixWheelDriveController.DriveMode.VELOCITY);

        double maxError = 0;
        double settlingTime = 0;
        boolean settled = false;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            double currentDistance = Math.abs(driveController.getX() - startX);
            double error = targetDistance - currentDistance;

            // Heading correction
            double currentHeading = driveController.getHeading();
            double headingError = normalizeAngle(startHeading - currentHeading);
            double headingCorrection = MovementTest.HEADING_CORRECTION_KP * headingError;

            if (Math.abs(error) > Math.abs(maxError)) {
                maxError = error;
            }

            if (!settled && Math.abs(error) < MovementTest.TOLERANCE) {
                settlingTime = timer.seconds();
                settled = true;
            }

            if (Math.abs(error) < MovementTest.TOLERANCE) {
                break;
            }

            if (timer.milliseconds() > MovementTest.TIMEOUT_MS) {
                break;
            }

            // Get base speed from PID
            double pidOutput = movePID.getOutput(timer.seconds(), -error);
            double speed = pidOutput;

            // DECELERATION ZONE - gradually reduce max speed near target
            double effectiveMaxSpeed = MovementTest.MAX_SPEED;
            if (Math.abs(error) < MovementTest.DECEL_DISTANCE) {
                double decelFactor = Math.abs(error) / MovementTest.DECEL_DISTANCE;
                decelFactor = decelFactor * decelFactor; // Quadratic for smooth curve
                effectiveMaxSpeed = MovementTest.MIN_SPEED +
                        (MovementTest.MAX_SPEED - MovementTest.MIN_SPEED) * decelFactor;
            }

            // Apply MIN_SPEED only when far from target
            if (Math.abs(speed) < MovementTest.MIN_SPEED && Math.abs(error) > MovementTest.TOLERANCE * 2) {
                speed = Math.signum(speed) * MovementTest.MIN_SPEED;
            }

            // Clamp to effective max speed
            speed = Math.max(-effectiveMaxSpeed, Math.min(effectiveMaxSpeed, speed));
            speed *= direction;

            // Apply heading correction
            double leftSpeed = speed - headingCorrection;
            double rightSpeed = speed + headingCorrection;

            leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
            rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

            driveController.tankDriveVelocityNormalized(leftSpeed, rightSpeed);

            sleep(10);
        }

        driveController.stopDrive();

        // Final results
        driveController.updateOdometry();
        double finalDistance = Math.abs(driveController.getX() - startX);
        double finalError = targetDistance - finalDistance;
        double finalHeadingError = normalizeAngle(startHeading - driveController.getHeading());

        if (settled) {
            telemetry.addData("Settling", "%.2fs", settlingTime);
        }
        telemetry.addData("Accuracy", "%.1f%%", 100 * (1 - Math.abs(finalError) / targetDistance));
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

            if (Math.abs(headingError) < toleranceRad) {
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

            sleep(10);
        }

        driveController.stopDrive();

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