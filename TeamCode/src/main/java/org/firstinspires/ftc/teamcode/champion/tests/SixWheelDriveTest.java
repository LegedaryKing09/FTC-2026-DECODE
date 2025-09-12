package org.firstinspires.ftc.teamcode.champion.tests;

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

    private SixWheelDriveController driveController = new SixWheelDriveController();

    // Direct motor references for debugging
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Controller...");
        telemetry.update();

        // Initialize the controller
        driveController.init(hardwareMap);

        // Get direct motor references for debugging
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "lf");
            frontRight = hardwareMap.get(DcMotor.class, "rf");
            backLeft = hardwareMap.get(DcMotor.class, "lb");
            backRight = hardwareMap.get(DcMotor.class, "rb");
            telemetry.addData("Status", "All motors found in hardware map");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to find motors: " + e.getMessage());
        }

        telemetry.addData("Status", "Hardware Initialized. Waiting for Start.");
        telemetry.addData("Controls", "Left stick = drive, Right stick = turn");
        telemetry.addData("Debug", "A = Test all motors, X = Test right side only");
        telemetry.addData("Debug", "Y = Test left side only, B = Individual motor test");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Normal driving controls
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                // Debug button tests
                if (gamepad1.a) {
                    // Test all motors at 50% power
                    testAllMotorsDirectly(0.5);
                    telemetry.addData("Test", "All motors at 50%");
                } else if (gamepad1.x) {
                    // Test right side motors only
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0.5);
                    backRight.setPower(0.5);
                    telemetry.addData("Test", "Right side motors only");
                } else if (gamepad1.y) {
                    // Test left side motors only
                    frontLeft.setPower(0.5);
                    backLeft.setPower(0.5);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    telemetry.addData("Test", "Left side motors only");
                } else if (gamepad1.b) {
                    // Individual motor test (cycle through motors)
                    long time = System.currentTimeMillis() / 1000;
                    int motorIndex = (int)(time % 4);

                    frontLeft.setPower(motorIndex == 0 ? 0.5 : 0);
                    frontRight.setPower(motorIndex == 1 ? 0.5 : 0);
                    backLeft.setPower(motorIndex == 2 ? 0.5 : 0);
                    backRight.setPower(motorIndex == 3 ? 0.5 : 0);

                    String[] motorNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
                    telemetry.addData("Test", "Testing: " + motorNames[motorIndex]);
                } else {
                    // Normal arcade drive
                    driveController.arcadeDrive(drive, turn);
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

                telemetry.addData("Expected Left Power", "%.2f", leftPower);
                telemetry.addData("Expected Right Power", "%.2f", rightPower);

                // Check if motors are actually connected and responsive
                telemetry.addData("Motor Status", "");
                telemetry.addData("FL Connected", frontLeft != null ? "Yes" : "No");
                telemetry.addData("FR Connected", frontRight != null ? "Yes" : "No");
                telemetry.addData("BL Connected", backLeft != null ? "Yes" : "No");
                telemetry.addData("BR Connected", backRight != null ? "Yes" : "No");

                // Show current motor powers
                if (frontLeft != null) telemetry.addData("FL Power", "%.2f", frontLeft.getPower());
                if (frontRight != null) telemetry.addData("FR Power", "%.2f", frontRight.getPower());
                if (backLeft != null) telemetry.addData("BL Power", "%.2f", backLeft.getPower());
                if (backRight != null) telemetry.addData("BR Power", "%.2f", backRight.getPower());

                telemetry.addData("Robot X", "%.2f", driveController.getX());
                telemetry.addData("Robot Y", "%.2f", driveController.getY());
                telemetry.addData("Heading (Degrees)", "%.2f", driveController.getHeadingDegrees());

                telemetry.update();
            }
        }

        driveController.stopDrive();
    }

    private void testAllMotorsDirectly(double power) {
        if (frontLeft != null) frontLeft.setPower(power);
        if (frontRight != null) frontRight.setPower(power);
        if (backLeft != null) backLeft.setPower(power);
        if (backRight != null) backRight.setPower(power);
    }
}