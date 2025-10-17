package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

/**
 * Test program specifically for velocity-based turning
 * Compare power mode vs velocity mode to diagnose issues
 */
@Config
@TeleOp(name = "Velocity Turn Test", group = "Testing")
public class VelocityTurnTest extends LinearOpMode {

    SixWheelDriveController driveController;

    // Test parameters
    @Config
    public static class TurnParams {
        public static boolean USE_VELOCITY_MODE = true;  // Toggle this to compare
        public static double TARGET_ANGLE = 90.0;
        public static double HEADING_TOLERANCE = 1.0;
        public static int STABLE_FRAMES_REQUIRED = 5;

        // Velocity mode parameters
        public static double MAX_ANGULAR_VELOCITY = 80.0;  // degrees/sec
        public static double MIN_ANGULAR_VELOCITY = 15.0;  // degrees/sec
        public static double SLOW_DOWN_ANGLE = 20.0;

        // Power mode parameters (for comparison)
        public static double MAX_TURN_POWER = 0.3;
        public static double MIN_TURN_POWER = 0.1;
    }

    private boolean testInProgress = false;
    private double initialHeading = 0;
    private double targetHeading = 0;

    @Override
    public void runOpMode() {
        // Initialize
        driveController = new SixWheelDriveController(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("=== VELOCITY TURN TEST ===");
        telemetry.addLine();
        telemetry.addData("Mode", TurnParams.USE_VELOCITY_MODE ? "VELOCITY" : "POWER");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("A: Reset Odometry");
        telemetry.addLine("B: Turn to target angle (BLOCKING)");
        telemetry.addLine("X: Stop motors");
        telemetry.addLine("D-Pad UP/DOWN: Adjust target angle");
        telemetry.addLine();
        telemetry.addLine("Use FTC Dashboard to change:");
        telemetry.addLine("- USE_VELOCITY_MODE (true/false)");
        telemetry.addLine("- MAX_ANGULAR_VELOCITY");
        telemetry.addLine("- TARGET_ANGLE");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        // Reset odometry at start
        driveController.resetOdometry();
        sleep(100);

        boolean lastA = false;
        boolean lastB = false;
        boolean lastX = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (opModeIsActive()) {
            driveController.updateOdometry();

            // Button A: Reset odometry
            if (gamepad1.a && !lastA) {
                driveController.resetOdometry();
                sleep(100);
                telemetry.addLine("✓ Odometry Reset");
                telemetry.update();
                sleep(500);
            }
            lastA = gamepad1.a;

            // Button B: Execute turn test
            if (gamepad1.b && !lastB && !testInProgress) {
                testInProgress = true;
                executeTurnTest();
                testInProgress = false;
            }
            lastB = gamepad1.b;

            // Button X: Emergency stop
            if (gamepad1.x && !lastX) {
                driveController.stopDrive();
                testInProgress = false;
                telemetry.addLine("⚠️ STOPPED");
                telemetry.update();
            }
            lastX = gamepad1.x;

            // D-Pad: Adjust target angle
            if (gamepad1.dpad_up && !lastDpadUp) {
                TurnParams.TARGET_ANGLE += 15.0;
                telemetry.addData("Target Angle", "%.1f°", TurnParams.TARGET_ANGLE);
                telemetry.update();
                sleep(200);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                TurnParams.TARGET_ANGLE -= 15.0;
                telemetry.addData("Target Angle", "%.1f°", TurnParams.TARGET_ANGLE);
                telemetry.update();
                sleep(200);
            }
            lastDpadDown = gamepad1.dpad_down;

            // Display status
            if (!testInProgress) {
                displayIdleStatus();
            }

            sleep(50);
        }
    }

    /**
     * Execute a blocking turn test using either velocity or power mode
     */
    private void executeTurnTest() {
        driveController.updateOdometry();
        initialHeading = driveController.getHeadingDegrees();
        targetHeading = normalizeAngle(initialHeading + TurnParams.TARGET_ANGLE);

        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║     TURN TEST STARTED         ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addData("Mode", TurnParams.USE_VELOCITY_MODE ? "VELOCITY" : "POWER");
        telemetry.addData("Initial Heading", "%.2f°", initialHeading);
        telemetry.addData("Target Heading", "%.2f°", targetHeading);
        telemetry.addData("Delta", "%.2f°", TurnParams.TARGET_ANGLE);
        telemetry.update();
        sleep(1000);

        long startTime = System.currentTimeMillis();
        int stableFrames = 0;
        int loopCount = 0;

        while (opModeIsActive() && testInProgress) {
            loopCount++;
            driveController.updateOdometry();
            double currentHeading = driveController.getHeadingDegrees();

            // Calculate error
            double error = angleDifference(currentHeading, targetHeading);
            double absError = Math.abs(error);

            // Display real-time data
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("Mode", TurnParams.USE_VELOCITY_MODE ? "VELOCITY" : "POWER");
            telemetry.addLine("═══════════════════════════════");
            telemetry.addData("Loop Count", loopCount);
            telemetry.addData("Time", "%.2fs", (System.currentTimeMillis() - startTime) / 1000.0);
            telemetry.addLine();
            telemetry.addData("Current Heading", "%.2f°", currentHeading);
            telemetry.addData("Target Heading", "%.2f°", targetHeading);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Abs Error", "%.2f°", absError);
            telemetry.addData("Tolerance", "%.2f°", TurnParams.HEADING_TOLERANCE);
            telemetry.addLine();

            // Check if aligned
            if (absError <= TurnParams.HEADING_TOLERANCE) {
                stableFrames++;
                telemetry.addLine("✓ WITHIN TOLERANCE");
                telemetry.addData("Stable Frames", "%d/%d", stableFrames, TurnParams.STABLE_FRAMES_REQUIRED);

                // STOP IMMEDIATELY
                driveController.stopDrive();

                if (stableFrames >= TurnParams.STABLE_FRAMES_REQUIRED) {
                    // SUCCESS!
                    double actualTurn = angleDifference(initialHeading, currentHeading);
                    double turnTime = (System.currentTimeMillis() - startTime) / 1000.0;

                    telemetry.addLine();
                    telemetry.addLine("╔═══════════════════════════════╗");
                    telemetry.addLine("║      ✓✓✓ ALIGNED ✓✓✓          ║");
                    telemetry.addLine("╚═══════════════════════════════╝");
                    telemetry.addData("Initial", "%.2f°", initialHeading);
                    telemetry.addData("Target", "%.2f°", targetHeading);
                    telemetry.addData("Final", "%.2f°", currentHeading);
                    telemetry.addData("Actual Turn", "%.2f°", actualTurn);
                    telemetry.addData("Final Error", "%.2f°", error);
                    telemetry.addData("Time", "%.2fs", turnTime);
                    telemetry.addData("Loops", loopCount);
                    telemetry.update();
                    sleep(3000);
                    return;
                }
            } else {
                stableFrames = 0;
                telemetry.addLine("⟳ TURNING...");

                // Apply turn command based on mode
                if (TurnParams.USE_VELOCITY_MODE) {
                    turnWithVelocity(error, absError);
                } else {
                    turnWithPower(error, absError);
                }
            }

            // Display wheel velocities
            telemetry.addLine();
            telemetry.addLine("─── WHEEL STATUS ───");
            telemetry.addData("Left Vel", "%.0f ticks/s", driveController.getLeftVelocity());
            telemetry.addData("Right Vel", "%.0f ticks/s", driveController.getRightVelocity());

            telemetry.update();

            sleep(20);
        }

        driveController.stopDrive();
    }

    /**
     * Turn using VELOCITY control
     */
    private void turnWithVelocity(double error, double absError) {
        double angularVelocity;

        if (absError > TurnParams.SLOW_DOWN_ANGLE) {
            // Far from target - max velocity
            angularVelocity = TurnParams.MAX_ANGULAR_VELOCITY;
        } else {
            // Close to target - proportional scaling
            double scale = absError / TurnParams.SLOW_DOWN_ANGLE;
            angularVelocity = TurnParams.MIN_ANGULAR_VELOCITY +
                    (TurnParams.MAX_ANGULAR_VELOCITY - TurnParams.MIN_ANGULAR_VELOCITY) * scale;
        }

        // Apply direction (error is positive for CCW, negative for CW)
        angularVelocity = Math.copySign(angularVelocity, error);

        // Command velocity
        // FIXED: Don't negate! If error is positive (need to turn left/CCW), velocity should be positive
        driveController.setAngularVelocity(angularVelocity);

        telemetry.addData("Angular Velocity", "%.1f °/s", angularVelocity);
        telemetry.addData("Error Sign", error > 0 ? "Positive (CCW)" : "Negative (CW)");
    }

    /**
     * Turn using POWER control (for comparison)
     */
    private void turnWithPower(double error, double absError) {
        double turnPower;

        if (absError > TurnParams.SLOW_DOWN_ANGLE) {
            // Far from target - max power
            turnPower = TurnParams.MAX_TURN_POWER;
        } else {
            // Close to target - proportional scaling
            double scale = absError / TurnParams.SLOW_DOWN_ANGLE;
            turnPower = TurnParams.MIN_TURN_POWER +
                    (TurnParams.MAX_TURN_POWER - TurnParams.MIN_TURN_POWER) * scale;
        }

        // Apply direction
        turnPower = Math.copySign(turnPower, error);

        // Tank drive for turning (left -power, right +power for CCW)
        driveController.tankDrive(-turnPower, turnPower);

        telemetry.addData("Turn Power", "%.3f", turnPower);
    }

    private void displayIdleStatus() {
        double currentHeading = driveController.getHeadingDegrees();

        telemetry.addLine("╔═══════════════════════════════╗");
        telemetry.addLine("║   VELOCITY TURN TEST - IDLE   ║");
        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.addLine();
        telemetry.addData("Mode", TurnParams.USE_VELOCITY_MODE ? "VELOCITY ⚙" : "POWER 🔋");
        telemetry.addData("Current Heading", "%.2f°", currentHeading);
        telemetry.addData("Target Delta", "%.1f°", TurnParams.TARGET_ANGLE);
        telemetry.addLine();

        if (TurnParams.USE_VELOCITY_MODE) {
            telemetry.addLine("─── VELOCITY SETTINGS ───");
            telemetry.addData("Max", "%.0f °/s", TurnParams.MAX_ANGULAR_VELOCITY);
            telemetry.addData("Min", "%.0f °/s", TurnParams.MIN_ANGULAR_VELOCITY);
            telemetry.addData("Slow Zone", "%.0f°", TurnParams.SLOW_DOWN_ANGLE);
        } else {
            telemetry.addLine("─── POWER SETTINGS ───");
            telemetry.addData("Max", "%.2f", TurnParams.MAX_TURN_POWER);
            telemetry.addData("Min", "%.2f", TurnParams.MIN_TURN_POWER);
            telemetry.addData("Slow Zone", "%.0f°", TurnParams.SLOW_DOWN_ANGLE);
        }

        telemetry.addLine();
        telemetry.addData("Tolerance", "%.1f°", TurnParams.HEADING_TOLERANCE);
        telemetry.addData("Stable Frames", "%d", TurnParams.STABLE_FRAMES_REQUIRED);
        telemetry.addLine();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("Press B to start turn test");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();
    }

    private double angleDifference(double current, double target) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}