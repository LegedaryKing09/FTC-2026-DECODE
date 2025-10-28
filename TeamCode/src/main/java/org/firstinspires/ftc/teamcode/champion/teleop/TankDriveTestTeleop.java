package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Tank Drive Test", group = "Test")
public class TankDriveTestTeleop extends OpMode {

    // Motor names matching existing configuration
    private static final String LF_NAME = "lf"; // Left Front
    private static final String RF_NAME = "rf"; // Right Front
    private static final String LB_NAME = "lb"; // Left Back
    private static final String RB_NAME = "rb"; // Right Back

    // Drive motors
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Tank Drive Test...");
        telemetry.update();

        try {
            // Initialize drive motors
            leftFront = hardwareMap.get(DcMotorEx.class, LF_NAME);
            rightFront = hardwareMap.get(DcMotorEx.class, RF_NAME);
            leftBack = hardwareMap.get(DcMotorEx.class, LB_NAME);
            rightBack = hardwareMap.get(DcMotorEx.class, RB_NAME);

            // Set motor directions to match existing configuration
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);

            // Set motors to brake mode for better control
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Run without encoders for simple tank drive
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("Status", "✓ Tank Drive Test Initialized");
            telemetry.addData("Controls", "Left Stick Y: Forward/Back | Right Stick X: Turn");
            telemetry.addData("Drive Motors", "%s, %s, %s, %s", LF_NAME, RF_NAME, LB_NAME, RB_NAME);

        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize: " + e.getMessage());
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        // Get joystick inputs - forward/backward and turning
        double drive = gamepad1.right_stick_x;  // Forward/backward movement
        double turn = -gamepad1.left_stick_y;   // Turning movement

        // Calculate motor powers using arcade drive math
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Normalize powers if they exceed 1.0 or -1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Apply power to motors
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        // Update telemetry
        telemetry.addData("Status", "Arcade Drive Active");
        telemetry.addData("Drive Power", "%.3f", drive);
        telemetry.addData("Turn Power", "%.3f", turn);
        telemetry.addData("Left Motors", "%.3f", leftPower);
        telemetry.addData("Right Motors", "%.3f", rightPower);
        telemetry.addData("Left Y", "%.3f", gamepad1.left_stick_y);
        telemetry.addData("Right X", "%.3f", gamepad1.right_stick_x);

        // Show gamepad inputs for debugging
        telemetry.addData("Gamepad Active", gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_x != 0.0);

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when teleop ends
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        telemetry.addData("Status", "Tank Drive Stopped");
        telemetry.update();
    }
}