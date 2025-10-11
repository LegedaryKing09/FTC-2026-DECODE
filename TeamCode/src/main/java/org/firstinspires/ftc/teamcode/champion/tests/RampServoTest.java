package org.firstinspires.ftc.teamcode.champion.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.RampController;

@Config
@TeleOp(name = "Ramp Servo Test", group = "Tests")
public class RampServoTest extends LinearOpMode {

    private RampController rampController;

    

    @Override
    public void runOpMode() throws InterruptedException {
        rampController = new RampController(this);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "");
        telemetry.addData("", "A: Min | B: Max | X: Center");
        telemetry.addData("", "Y: Quarter | RB: Three Quarter");
        telemetry.addData("", "DPad Up: 0° | DPad Right: 60°");
        telemetry.addData("", "DPad Down: 120° | DPad Left: 180°");
        telemetry.addData("", "LB: 240° | LT/RT: Inc/Dec 10°");
        telemetry.addData("", "Left Stick Y: Manual Position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Test preset positions (0.0 to 1.0)
            if (gamepad1.a) {
                rampController.setToMin();
                telemetry.addData("Action", "Min Position");
            } else if (gamepad1.b) {
                rampController.setToMax();
                telemetry.addData("Action", "Max Position");
            } else if (gamepad1.x) {
                rampController.setToCenter();
                telemetry.addData("Action", "Center Position");
            } else if (gamepad1.y) {
                rampController.setToQuarter();
                telemetry.addData("Action", "Quarter Position");
            } else if (gamepad1.right_bumper) {
                rampController.setToThreeQuarter();
                telemetry.addData("Action", "Three Quarter Position");
            }

            // Test preset angles (0° to 240°)
            else if (gamepad1.dpad_up) {
                rampController.setTo0Degrees();
                telemetry.addData("Action", "0 Degrees");
            } else if (gamepad1.dpad_right) {
                rampController.setTo60Degrees();
                telemetry.addData("Action", "60 Degrees");
            } else if (gamepad1.dpad_down) {
                rampController.setTo120Degrees();
                telemetry.addData("Action", "120 Degrees");
            } else if (gamepad1.dpad_left) {
                rampController.setTo180Degrees();
                telemetry.addData("Action", "180 Degrees");
            } else if (gamepad1.left_bumper) {
                rampController.setTo240Degrees();
                telemetry.addData("Action", "240 Degrees");
            }

            // Test incremental angle movement
            else if (gamepad1.left_trigger > 0.5) {
                rampController.decrementAngle(10.0);
                telemetry.addData("Action", "Decrement 10°");
            } else if (gamepad1.right_trigger > 0.5) {
                rampController.incrementAngle(10.0);
                telemetry.addData("Action", "Increment 10°");
            }

            // Manual position control with left stick
            else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                double manualPosition = (-gamepad1.left_stick_y + 1) / 2.0; // Map -1 to 1 to 0 to 1 (inverted for intuitive control)
                rampController.setPosition(manualPosition);
                telemetry.addData("Action", "Manual: %.2f", manualPosition);
            }

            // Display current state
            telemetry.addData("", "--- Current State ---");
            telemetry.addData("Position", "%.3f", rampController.getPosition());
            telemetry.addData("Angle", "%.1f°", rampController.getAngle());

            // Display status checks
            telemetry.addData("", "--- Status Checks ---");
            telemetry.addData("At Min?", rampController.isAtMinPosition());
            telemetry.addData("At Max?", rampController.isAtMaxPosition());
            telemetry.addData("At Center?", rampController.isAtCenter());
            telemetry.addData("At 120° (±5°)?", rampController.isAtAngle(120.0, 5.0));

            telemetry.update();

            sleep(100); // Small delay to prevent overwhelming the servo
        }
    }
}