package org.firstinspires.ftc.teamcode.champion.Auton.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

@Autonomous(name = "PID Turn 90 CCW v2", group = "Autonomous")
public class NewTurningTest extends LinearOpMode {

    private SixWheelDriveController driveController;

    // PID constants
    private static final double kP = 0.02;
    private static final double kI = 0.0;      // Start with 0, add later if needed
    private static final double kD = 0.003;

    // Turn parameters
    private static final double TARGET_ANGLE = 90.0;
    private static final double ANGLE_TOLERANCE = 2.0;
    private static final double MAX_TURN_POWER = 0.4;
    private static final double MIN_TURN_POWER = 0.15;
    private static final double TIMEOUT_SECONDS = 5.0;
    private static final int SETTLE_COUNT_TARGET = 10;

    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        driveController = new SixWheelDriveController(this);

        telemetry.addData("Status", "Initialized - PID Turn 90° CCW");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveController.updateOdometry();
            double startHeading = driveController.getHeadingDegrees();

            // TARGET: For CCW 90°, we need heading to change by +90 or -90
            // depending on your sensor convention.
            // TRY THIS FIRST: If CCW makes heading MORE NEGATIVE:
            double targetHeading = normalizeAngle(startHeading - TARGET_ANGLE);

            // IF THAT DOESN'T WORK, use this instead:
            // double targetHeading = normalizeAngle(startHeading + TARGET_ANGLE);

            telemetry.addData("Start", "%.2f°", startHeading);
            telemetry.addData("Target", "%.2f°", targetHeading);
            telemetry.update();
            sleep(500);

            integral = 0;
            lastError = 0;
            pidTimer.reset();

            ElapsedTime timeoutTimer = new ElapsedTime();
            int settleCount = 0;

            while (opModeIsActive() && timeoutTimer.seconds() < TIMEOUT_SECONDS) {
                driveController.updateOdometry();
                double currentHeading = driveController.getHeadingDegrees();

                // Calculate error - shortest path
                double error = normalizeAngle(targetHeading - currentHeading);

                // Check tolerance
                if (Math.abs(error) <= ANGLE_TOLERANCE) {
                    settleCount++;
                    if (settleCount >= SETTLE_COUNT_TARGET) {
                        break;
                    }
                } else {
                    settleCount = 0;
                }

                // PID calculation
                double turnPower = calculatePID(error);

                // CRITICAL: Determine turn direction based on error sign
                // If error is positive, we need to turn in the direction that
                // INCREASES heading toward target
                // If error is negative, we need to turn the other way

                // In your controller:
                // positive turn = left+, right- = clockwise (typically)
                // negative turn = left-, right+ = counter-clockwise (typically)

                // So if error > 0 and we need CCW to fix it, send negative turn
                // This depends on your heading convention!

                // TRY THIS: Direct mapping (error drives turn directly)
                driveController.arcadeDrive(0, turnPower);

                telemetry.addData("Current", "%.2f°", currentHeading);
                telemetry.addData("Target", "%.2f°", targetHeading);
                telemetry.addData("Error", "%.2f°", error);
                telemetry.addData("Turn Power", "%.3f", turnPower);
                telemetry.addData("Settle", "%d/%d", settleCount, SETTLE_COUNT_TARGET);
                telemetry.addData("Motors", driveController.getMotorPowers());
                telemetry.update();
            }

            driveController.stopDrive();
            sleep(300);

            driveController.updateOdometry();
            double finalHeading = driveController.getHeadingDegrees();

            telemetry.addData("=== DONE ===", "");
            telemetry.addData("Start", "%.2f°", startHeading);
            telemetry.addData("Final", "%.2f°", finalHeading);
            telemetry.addData("Actual Turn", "%.2f°", normalizeAngle(finalHeading - startHeading));
            telemetry.addData("Target was", "90° CCW");
            telemetry.update();
            sleep(5000);
        }
    }

    private double calculatePID(double error) {
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();
        if (deltaTime == 0) deltaTime = 0.02;

        double pTerm = kP * error;

        integral += error * deltaTime;
        double maxIntegral = MAX_TURN_POWER / (kI + 0.001);
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double iTerm = kI * integral;

        double derivative = (error - lastError) / deltaTime;
        double dTerm = kD * derivative;
        lastError = error;

        double output = pTerm + iTerm + dTerm;

        // Minimum power to overcome friction
        if (Math.abs(error) > ANGLE_TOLERANCE) {
            if (output > 0 && output < MIN_TURN_POWER) {
                output = MIN_TURN_POWER;
            } else if (output < 0 && output > -MIN_TURN_POWER) {
                output = -MIN_TURN_POWER;
            }
        }

        return Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, output));
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}