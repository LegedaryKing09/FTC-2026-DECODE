package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Dual Servo Test", group = "Tests")
public class ZeroingProgram extends LinearOpMode {

    private Servo servo1;
    private Servo servo2;

    private double currentPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servos - update these names to match your hardware config
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "");
        telemetry.addData("", "A: Position = 1.0 (Max)");
        telemetry.addData("", "B: Position = 0.75");
        telemetry.addData("", "X: Position = 0.5 (Center) - HOLD");
        telemetry.addData("", "Y: Position = 0.25");
        telemetry.addData("", "DPad Up: Position = 0.0 (Min)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean positionChanged = false;

            // Position control buttons
            if (gamepad1.a) {
                currentPosition = 1.0;
                positionChanged = true;
                telemetry.addData("Action", "Set Position to 1.0 (Max)");
            } else if (gamepad1.b) {
                currentPosition = 0.75;
                positionChanged = true;
                telemetry.addData("Action", "Set Position to 0.75");
            } else if (gamepad1.x) {
                currentPosition = 0.5;
                positionChanged = true;
                telemetry.addData("Action", "Set Position to 0.5 (Center)");
            } else if (gamepad1.y) {
                currentPosition = 0.25;
                positionChanged = true;
                telemetry.addData("Action", "Set Position to 0.25");
            } else if (gamepad1.dpad_up) {
                currentPosition = 0.0;
                positionChanged = true;
                telemetry.addData("Action", "Set Position to 0.0 (Min)");
            }

            // Apply position to both servos when changed
            if (positionChanged) {
                servo1.setPosition(currentPosition);
                servo2.setPosition(currentPosition);
            }

            // Display current state
            telemetry.addData("", "--- Current State ---");
            telemetry.addData("Current Position", "%.2f", currentPosition);
            telemetry.addData("Servo 1 Position", "%.2f", servo1.getPosition());
            telemetry.addData("Servo 2 Position", "%.2f", servo2.getPosition());
            telemetry.addData("Servo 1 Direction", servo1.getDirection());
            telemetry.addData("Servo 2 Direction", servo2.getDirection());

            telemetry.update();

            sleep(100); // Small delay to prevent button spamming
        }
    }
}