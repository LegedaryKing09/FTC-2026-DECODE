package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Motor Balance")
public class TestMotorBalance extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rb");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Left side only
        telemetry.addData("Test", "LEFT SIDE ONLY");
        telemetry.addData("Power", "50%");
        telemetry.update();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        sleep(3000);
        leftFront.setPower(0);
        leftBack.setPower(0);

        int leftTicks = (leftFront.getCurrentPosition() + leftBack.getCurrentPosition()) / 2;

        sleep(2000);

        // Right side only
        telemetry.addData("Test", "RIGHT SIDE ONLY");
        telemetry.update();

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setPower(0.5);
        rightBack.setPower(0.5);
        sleep(3000);
        rightFront.setPower(0);
        rightBack.setPower(0);

        int rightTicks = (rightFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        // Compare
        double difference = Math.abs(leftTicks - rightTicks) / (double)Math.max(leftTicks, rightTicks) * 100.0;

        telemetry.clear();
        telemetry.addData("=== RESULTS ===", "");
        telemetry.addData("Left Ticks (3 sec)", leftTicks);
        telemetry.addData("Right Ticks (3 sec)", rightTicks);
        telemetry.addData("Difference", String.format("%.1f%%", difference));
        telemetry.addData("", "");

        if (difference < 3) {
            telemetry.addData("Status", "✓ Motors well matched");
        } else if (difference < 7) {
            telemetry.addData("Status", "⚠ Acceptable mismatch");
            telemetry.addData("This causes", "slight drift");
        } else {
            telemetry.addData("Status", "⚠⚠ SIGNIFICANT MISMATCH");
            telemetry.addData("This causes", "heavy drift!");

            if (leftTicks > rightTicks) {
                telemetry.addData("Issue", "Left motors STRONGER");
                telemetry.addData("Result", "Robot drifts RIGHT");
            } else {
                telemetry.addData("Issue", "Right motors STRONGER");
                telemetry.addData("Result", "Robot drifts LEFT");
            }
        }

        telemetry.update();
        sleep(60000);
    }
}