package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Simple Drive OpMode
 *
 * Motor Configuration:
 * L1: FORWARD
 * L2: REVERSE
 * L3: FORWARD
 * R1: REVERSE
 * R2: FORWARD
 * R3: FORWARD
 *
 * Controls:
 * Left Stick Y: Drive forward/backward
 * Right Stick X: Turn left/right
 */
@Config
@TeleOp(name = "Simple Drive", group = "Test")
public class SimpleDrive extends LinearOpMode {

    // Motor names
    public static String L1_NAME = "lf";
    public static String L2_NAME = "lb";
    public static String R1_NAME = "rf";
    public static String R2_NAME = "rb";

    private DcMotor l1, l2, r1, r2;

    @Override
    public void runOpMode() {
        // Initialize motors
        l1 = hardwareMap.get(DcMotor.class, L1_NAME);
        l2 = hardwareMap.get(DcMotor.class, L2_NAME);
        r1 = hardwareMap.get(DcMotor.class, R1_NAME);
        r2 = hardwareMap.get(DcMotor.class, R2_NAME);

        // Set directions: L1:F, L2:R, L3:F, R1:R, R2:F, R3:F
        l1.setDirection(DcMotorSimple.Direction.FORWARD);
        l2.setDirection(DcMotorSimple.Direction.REVERSE);
        r1.setDirection(DcMotorSimple.Direction.REVERSE);
        r2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake mode
        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== SIMPLE DRIVE ===");
        telemetry.addLine("Left Stick Y: Drive");
        telemetry.addLine("Right Stick X: Turn");
        telemetry.addLine("FULL POWER");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double turn = gamepad1.right_stick_x;   // Left/right turn

            // Calculate motor powers (arcade drive)
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Clip to -1 to 1
            leftPower = Math.max(-1, Math.min(1, leftPower));
            rightPower = Math.max(-1, Math.min(1, rightPower));

            // Set motor powers
            l1.setPower(leftPower);
            l2.setPower(leftPower);

            r1.setPower(rightPower);
            r2.setPower(rightPower);

            // Telemetry
            telemetry.addLine("=== SIMPLE DRIVE ===");
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addLine();
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.update();
        }
    }
}