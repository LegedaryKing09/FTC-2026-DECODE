package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.ServoController;

@TeleOp(name = "Servo Test", group = "Tests")
public class ServoTest extends LinearOpMode {

    private ServoController servoController;
    private static final double MAX_DEGREES = 180.0;
    private static final double INCREMENT_DEGREES = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        servoController = new ServoController(this);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "");
        telemetry.addData("", "DPad Up: +5° | DPad Down: -5°");
        telemetry.addData("", "A: Min (0°) | B: Max (180°)");
        telemetry.addData("", "X: Center (90°)");
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
            }

            // Display current state
            telemetry.addData("", "--- Current State ---");
            telemetry.addData("Position", "%.3f", servoController.getPosition());
            telemetry.addData("Angle", "%.1f°", getAngle());

            telemetry.update();

            sleep(100); // Small delay to prevent overwhelming the servo
        }
    }

    private double getAngle() {
        return (1.0 - servoController.getPosition()) * MAX_DEGREES;
    }

    private void setAngle(double angle) {
        // Clamp angle to valid range
        angle = Math.max(0.0, Math.min(MAX_DEGREES, angle));
        double position = 1.0 - (angle / MAX_DEGREES);
        servoController.setPosition(position);
    }

    private void incrementAngle(double degrees) {
        double currentAngle = getAngle();
        setAngle(currentAngle + degrees);
    }
}