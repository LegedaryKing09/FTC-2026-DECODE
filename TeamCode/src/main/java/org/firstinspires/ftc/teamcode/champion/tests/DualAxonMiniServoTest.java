package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.AxonMiniServoController;

@TeleOp(name = "Dual AXON Mini Servo Test", group = "Tests")
public class DualAxonMiniServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AxonMiniServoController axonMiniServoController = new AxonMiniServoController(this);

        telemetry.addData("Status", "Initialized - Second servo: " + (axonMiniServoController.hasSecondServo() ? "Present" : "Not Found"));
        telemetry.addData("Controls", "");
        telemetry.addData("", "A: Run both servos at full power (opposite directions)");
        telemetry.addData("", "B: Stop both servos");
        telemetry.addData("", "X: Run first servo forward, second reverse");
        telemetry.addData("", "Y: Run first servo reverse, second forward");
        telemetry.update();

        waitForStart();

        boolean isRunningFullPower = false;

        while (opModeIsActive()) {
            // Run both servos at full power in opposite directions
            if (gamepad1.a) {
                isRunningFullPower = true;
                axonMiniServoController.forwardFull(); // First servo max, second servo min (due to reverse direction)
                telemetry.addData("Action", "Running both at full power - Opposite directions");
            } else if (gamepad1.b) {
                isRunningFullPower = false;
                axonMiniServoController.stop();
                telemetry.addData("Action", "Stopped both servos");
            } else if (gamepad1.x) {
                isRunningFullPower = false;
                axonMiniServoController.forwardFull();
                telemetry.addData("Action", "First forward, second reverse");
            } else if (gamepad1.y) {
                isRunningFullPower = false;
                axonMiniServoController.reverseFull();
                telemetry.addData("Action", "First reverse, second forward");
            }

            // Display current state
            telemetry.addData("", "--- Current State ---");
            telemetry.addData("First Servo Position", "%.3f", axonMiniServoController.getPosition());
            telemetry.addData("First Servo Power", "%.2f", axonMiniServoController.getPower());
            telemetry.addData("First Moving Forward?", axonMiniServoController.isMovingForward());
            telemetry.addData("First Moving Reverse?", axonMiniServoController.isMovingReverse());
            telemetry.addData("First Stopped?", axonMiniServoController.isStopped());

            if (axonMiniServoController.hasSecondServo()) {
                telemetry.addData("Second Servo Present", "Yes");
                telemetry.addData("Second Servo Position", "%.3f", axonMiniServoController.getSecondServo().getPosition());
                double secondPower = (axonMiniServoController.getSecondServo().getPosition() * 2.0) - 1.0;
                telemetry.addData("Second Servo Power", "%.2f", secondPower);
                telemetry.addData("Second Moving Forward?", secondPower > 0);
                telemetry.addData("Second Moving Reverse?", secondPower < 0);
                telemetry.addData("Second Stopped?", secondPower == 0);
            } else {
                telemetry.addData("Second Servo Present", "No");
            }

            if (isRunningFullPower) {
                telemetry.addData("Status", "Running at full power - opposite directions");
            }

            telemetry.update();

            sleep(100); // Small delay to prevent overwhelming the servos
        }
    }
}