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
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
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
            telemetry.addData("Controls", "Left Stick: Left Motors | Right Stick: Right Motors");
            telemetry.addData("Left Motors", "%s, %s", LF_NAME, LB_NAME);
            telemetry.addData("Right Motors", "%s, %s", RF_NAME, RB_NAME);

        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize: " + e.getMessage());
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        // Get joystick inputs
        double leftPower = -gamepad1.left_stick_y;  // Negative because up is negative Y
        double rightPower = -gamepad1.right_stick_y; // Negative because up is negative Y

        // Apply power to motors
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        // Update telemetry
        telemetry.addData("Status", "Tank Drive Active");
        telemetry.addData("Left Power", "%.3f", leftPower);
        telemetry.addData("Right Power", "%.3f", rightPower);
        telemetry.addData("Left Y", "%.3f", gamepad1.left_stick_y);
        telemetry.addData("Right Y", "%.3f", gamepad1.right_stick_y);

        // Show gamepad inputs for debugging
        telemetry.addData("Gamepad Active", gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_y != 0.0);

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