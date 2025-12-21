package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Dual CRServo Test", group = "Tests")
public class AxonMiniServoTest extends LinearOpMode {

    private CRServo servo1;
    private CRServo servo2;

    private double currentPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servos - update these names to match your hardware config
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "");
        telemetry.addData("", "A: Power = 1.0 (Full Forward)");
        telemetry.addData("", "B: Power = 0.5 (Half Forward)");
        telemetry.addData("", "X: Power = 0.0 (Stop)");
        telemetry.addData("", "Y: Power = -0.5 (Half Reverse)");
        telemetry.addData("", "DPad Up: Power = -1.0 (Full Reverse)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean powerChanged = false;

            // Power control buttons
            if (gamepad1.a) {
                currentPower = 1.0;
                powerChanged = true;
                telemetry.addData("Action", "Set Power to 1.0 (Full Forward)");
            } else if (gamepad1.b) {
                currentPower = 0.5;
                powerChanged = true;
                telemetry.addData("Action", "Set Power to 0.5 (Half Forward)");
            } else if (gamepad1.x) {
                currentPower = 0.0;
                powerChanged = true;
                telemetry.addData("Action", "Set Power to 0.0 (Stop)");
            } else if (gamepad1.y) {
                currentPower = -0.5;
                powerChanged = true;
                telemetry.addData("Action", "Set Power to -0.5 (Half Reverse)");
            } else if (gamepad1.dpad_up) {
                currentPower = -1.0;
                powerChanged = true;
                telemetry.addData("Action", "Set Power to -1.0 (Full Reverse)");
            }

            // Apply power to both servos when changed
            if (powerChanged) {
                servo1.setPower(currentPower);
                servo2.setPower(currentPower);
            }

            // Display current state
            telemetry.addData("", "--- Current State ---");
            telemetry.addData("Current Power", "%.2f", currentPower);
            telemetry.addData("Servo 1 Power", "%.2f", servo1.getPower());
            telemetry.addData("Servo 2 Power", "%.2f", servo2.getPower());
            telemetry.addData("Servo 1 Direction", servo1.getDirection());
            telemetry.addData("Servo 2 Direction", servo2.getDirection());

            telemetry.update();

            sleep(100); // Small delay to prevent button spamming
        }
    }
}