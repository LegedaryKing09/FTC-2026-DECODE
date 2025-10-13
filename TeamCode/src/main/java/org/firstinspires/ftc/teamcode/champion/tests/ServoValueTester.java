package org.firstinspires.ftc.teamcode.champion.tests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoValueTester")
//makes a servo go forward based on the y axis of the left joystick
public class ServoValueTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        boolean pressB = false;
        boolean pressX = false;
        boolean pressY = false;
        boolean pressA = false;
        while (opModeIsActive()) {
            double position = servo.getPosition();
            if (gamepad1.b && !pressB) { //if pressing b and var is false
                servo.setPosition(position + 0.01);
                pressB = true;
            }
            else if (!gamepad1.b && pressB) {
                pressB = false;
            }

            if (gamepad1.x && !pressX) {
                servo.setPosition(position - 0.01);
                pressX = true;
            }
            else if (!gamepad1.x && pressX) {
                pressX = false;
            }

            if (gamepad1.y && !pressY) { //if pressing y and var is false
                servo.setPosition(position + 0.1);
                pressY = true;
            }
            else if (!gamepad1.y && pressY) {
                pressY = false;
            }

            if (gamepad1.a && !pressA) {
                servo.setPosition(position - 0.1);
                pressA = true;
            }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }

            telemetry.addData("position:" , servo.getPosition());telemetry.update();
        }

    }
}
