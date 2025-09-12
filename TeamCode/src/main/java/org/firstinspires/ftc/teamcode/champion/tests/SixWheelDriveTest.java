package org.firstinspires.ftc.teamcode.champion.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.champion.controller.SixWheelDriveController;

/**
 * Enhanced test OpMode with comprehensive motor debugging capabilities.
 * This will help identify if the issue is hardware or software related.
 */
@TeleOp(name = "Debug Six-Wheel Drive", group = "Tests")
public class SixWheelDriveTest extends LinearOpMode {

    private SixWheelDriveController driveController;

    boolean isPressingLeftBumper = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Controller...");
        telemetry.update();

        // Initialize the controller
        driveController = new SixWheelDriveController(this);

        telemetry.addData("Status", "Hardware Initialized. Waiting for Start.");
        telemetry.addData("Controls", "Left stick = drive, Right stick = turn");
        telemetry.addData("Debug", "A = Test all motors, X = Test right side only");
        telemetry.addData("Debug", "Y = Test left side only, B = Individual motor test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Normal driving controls
            double drive = -gamepad1.left_stick_y * driveController.SLOW_SPEED_MULTIPLIER;
            double turn = gamepad1.right_stick_x * driveController.SLOW_TURN_MULTIPLIER;

            waitForStart();
            // Drivetrain Power Control
            if (gamepad1.left_bumper && !isPressingLeftBumper) {
                isPressingLeftBumper = true;
                if (driveController.isFastSpeedMode()) {
                    driveController.setSlowSpeed();
                } else {
                    driveController.setFastSpeed();
                }
            } else if (!gamepad1.left_bumper && isPressingLeftBumper) {
                isPressingLeftBumper = false;
            }

            // Debug button tests
            if (gamepad1.a) {

                // Test all motors at 50% power
                driveController.testAllMotorsDirectly(0.5);
                telemetry.addData("Test", "All motors at 50%");
            } else if (gamepad1.x) {

                // Test right side motors only
                driveController.setLeftPower(0);
                driveController.setRightPower(0.5);
                telemetry.addData("Test", "Right side motors only");
            } else if (gamepad1.y) {

                // Test left side motors only
                driveController.setLeftPower(0.5);
                driveController.setRightPower(0);
                telemetry.addData("Test", "Left side motors only");
            } else if (gamepad1.b) {
                // Individual motor test (cycle through motors)
                long time = System.currentTimeMillis() / 1000;
                int motorIndex = (int) (time % 4);

                driveController.setFrontLeftPower(motorIndex == 0 ? 0.5 : 0);
                driveController.setFrontRightPower(motorIndex == 1 ? 0.5 : 0);
                driveController.setBackLeftPower(motorIndex == 2 ? 0.5 : 0);
                driveController.setBackRightPower(motorIndex == 3 ? 0.5 : 0);

                String[] motorNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
                telemetry.addData("Test", "Testing: " + motorNames[motorIndex]);
            } else {
                // Normal arcade drive
                if (!driveController.isFastSpeedMode()) {
                    driveController.arcadeDrive(drive, turn);
                } else {
                    driveController.arcadeDrive(drive, turn);
                }
            }

            // Update odometry
            driveController.updateOdometry();

            // Display telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Drive Input", "%.2f", drive);
            telemetry.addData("Turn Input", "%.2f", turn);

            // Calculate what the motor powers should be
            double leftPower = drive + turn;
            double rightPower = drive - turn;
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            driveController.giveAllTelemetry();

            telemetry.addData("Expected Left Power", "%.2f", leftPower);
            telemetry.addData("Expected Right Power", "%.2f", rightPower);

            telemetry.addData("Robot X", "%.2f", driveController.getX());
            telemetry.addData("Robot Y", "%.2f", driveController.getY());
            telemetry.addData("Heading (Degrees)", "%.2f", driveController.getHeadingDegrees());

            telemetry.update();
        }
        driveController.stopDrive();
    }

}