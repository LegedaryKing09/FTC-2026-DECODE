package org.firstinspires.ftc.teamcode.champion.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AxonMiniServoTest", group = "Test")
public class AxonServoTest extends LinearOpMode {

    private Servo testServo;
    private double servoPosition = 0.5;

    private static final double DEGREE_90 = 0.25;

    private boolean lastAPressed = false;
    private boolean lastBPressed = false;

    @Override
    public void runOpMode() {
        testServo = hardwareMap.get(Servo.class, "servo");
        testServo.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Position", servoPosition);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean aPressed = gamepad1.a;
            boolean bPressed = gamepad1.b;

            if (aPressed && !lastAPressed) {
                servoPosition += DEGREE_90;
                if (servoPosition > 1.0) servoPosition = 1.0;
                testServo.setPosition(servoPosition);
            }

            if (bPressed && !lastBPressed) {
                servoPosition -= DEGREE_90;
                if (servoPosition < 0.0) servoPosition = 0.0;
                testServo.setPosition(servoPosition);
            }

            lastAPressed = aPressed;
            lastBPressed = bPressed;

            telemetry.addData("Commanded Position", "%.2f", servoPosition);
            telemetry.addData("Commanded Degrees", "%.0f°", servoPosition * 180);
            telemetry.addLine();
            telemetry.addData("A Pressed", aPressed);
            telemetry.addData("B Pressed", bPressed);
            telemetry.addData("Controls", "A = +90° CW | B = -90° CCW");
            telemetry.update();
        }
    }
}