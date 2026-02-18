package org.firstinspires.ftc.teamcode.champion.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Tank Drive - Each joystick controls one side
 *
 * Left Stick Y:  Left motors (L1, L2, L3)
 * Right Stick Y: Right motors (R1, R2, R3)
 *
 * Push both forward = drive forward
 * Push both backward = drive backward
 * Left forward + Right backward = turn right
 * Left backward + Right forward = turn left
 */
@Config
@TeleOp(name = "Tank Drive", group = "Test")
public class TankDrive extends LinearOpMode {

    // Motor names
    public static String L1_NAME = "lf";
    public static String L2_NAME = "lb";
    public static String R1_NAME = "rf";
    public static String R2_NAME = "rb";


    // Motor directions: L1:F, L2:R, L3:F, R1:R, R2:F, R3:F
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


        telemetry.addLine("=== TANK DRIVE ===");
        telemetry.addLine("Left Stick Y = Left motors");
        telemetry.addLine("Right Stick Y = Right motors");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs (negate Y because joystick Y is inverted)
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            // Set motor powers
            l1.setPower(leftPower);
            l2.setPower(leftPower);

            r1.setPower(rightPower);
            r2.setPower(rightPower);


            // Telemetry
            telemetry.addLine("=== TANK DRIVE ===");
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.update();
        }
    }
}