package org.firstinspires.ftc.teamcode.champion.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CRServoTester")
//makes a servo go forward based on the y axis of the left joystick
public class CRServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();

        boolean pressB = false;
        boolean pressX = false;
        boolean pressY = false;
        boolean pressA = false;
        boolean pressDpadDown = false;
        boolean pressDpadUp = false;
        while (opModeIsActive()) {
            double position = servo.getPower();

            if (gamepad1.x && !pressX) {
                servo.setPower(0);
                pressX = true;
            }
            else if (!gamepad1.x && pressX) {
                pressX = false;
            }

    if (gamepad1.a && !pressA) {
                servo.setPower(-1);
                pressA = true;
            }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }

            if (gamepad1.y && !pressY) {
                servo.setPower(1);
                pressY = true;
            }
            else if (!gamepad1.y && pressY) {
                pressY = false;
            }

            if (gamepad1.dpad_down && !pressDpadDown) {
                servo.setPower(servo.getPower() - 0.1);
                pressDpadDown = true;
            }
            else if (!gamepad1.dpad_down && pressDpadDown) {
                pressDpadDown = false;
            }

            if (gamepad1.dpad_up && !pressDpadUp) {
                servo.setPower(servo.getPower() + 0.1);
                pressDpadUp = true;
            }
            else if (!gamepad1.dpad_up && pressDpadUp) {
                pressDpadUp = false;
            }

            telemetry.addLine("A  to set power -1");
            telemetry.addLine("X to stop");
            telemetry.addLine("Y to set power 1");
            telemetry.addLine("DpadUp and Dpaddown to change power by 0.1 ");
            telemetry.addLine("Plug into servo port 0 and make sure the metal faces the numbers ");
            telemetry.addLine("Change config to 'servo' if not already set ");
            telemetry.addData("power:" , servo.getPower());telemetry.update();
        }

    }
}
