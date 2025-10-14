package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.AxonMiniServoController;

@TeleOp(name = "Servo Test", group = "Tests")
public class AxonMiniServoTest extends LinearOpMode {

    private AxonMiniServoController axonMiniServoController;
    private static final double MAX_DEGREES = 180.0;
    private static final double INCREMENT_DEGREES = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {
        axonMiniServoController = new AxonMiniServoController(this);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "");
        telemetry.addData("", "DPad Up: +5° | DPad Down: -5°");
        telemetry.addData("", "A: Min (0°) | B: Max (180°)");
        telemetry.addData("", "X: Center (90°) | Y: Test 0.1 | RB: Test 0.9");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Increment/Decrement with DPad
            if (gamepad1.dpad_up) {
                incrementAngle(INCREMENT_DEGREES);
                telemetry.addData("Action", "+%.0f°", INCREMENT_DEGREES);
            } else if (gamepad1.dpad_down) {
                incrementAngle(-INCREMENT_DEGREES);
                telemetry.addData("Action", "-%.0f°", INCREMENT_DEGREES);
            }

            // Preset positions
            else if (gamepad1.a) {
                setAngle(0.0);
                telemetry.addData("Action", "Min Position (0°)");
            } else if (gamepad1.b) {
                setAngle(MAX_DEGREES);
                telemetry.addData("Action", "Max Position (180°)");
            } else if (gamepad1.x) {
                setAngle(MAX_DEGREES / 2);
                telemetry.addData("Action", "Center Position (90°)");
            } else if (gamepad1.y) {
                // Test edge positions to check if servo can move beyond normal range
                axonMiniServoController.setPosition(0.1);
                telemetry.addData("Action", "Test Low Position (0.1)");
            } else if (gamepad1.right_bumper) {
                // Test edge positions to check if servo can move beyond normal range
                axonMiniServoController.setPosition(0.9);
                telemetry.addData("Action", "Test High Position (0.9)");
            }

            // Display current state
            telemetry.addData("", "--- Current State ---");
            telemetry.addData("Position", "%.3f", axonMiniServoController.getPosition());
            telemetry.addData("Angle", "%.1f°", getAngle());
            telemetry.addData("Servo Direction", axonMiniServoController.getServo().getDirection());
            telemetry.addData("Is Moving Forward?", axonMiniServoController.isMovingForward());
            telemetry.addData("Is Moving Reverse?", axonMiniServoController.isMovingReverse());
            telemetry.addData("Is Stopped?", axonMiniServoController.isStopped());
            telemetry.addData("Power Equivalent", "%.2f", axonMiniServoController.getPower());

            telemetry.update();

            sleep(100); // Small delay to prevent overwhelming the servo
        }
    }

    private double getAngle() {
        return (1.0 - axonMiniServoController.getPosition()) * MAX_DEGREES;
    }

    private void setAngle(double angle) {
        // Clamp angle to valid range
        angle = Math.max(0.0, Math.min(MAX_DEGREES, angle));
        double position = 1.0 - (angle / MAX_DEGREES);
        axonMiniServoController.setPosition(position);
    }

    private void incrementAngle(double degrees) {
        double currentAngle = getAngle();
        setAngle(currentAngle + degrees);
    }
}