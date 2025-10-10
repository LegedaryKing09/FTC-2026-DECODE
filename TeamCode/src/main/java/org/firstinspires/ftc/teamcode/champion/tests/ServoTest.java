package org.firstinspires.ftc.teamcode.champion.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.champion.controller.ServoController;

@TeleOp(name = "Servo Test", group = "Tests")
public class ServoTest extends LinearOpMode {

    private ServoController servoController;

    @Override
    public void runOpMode() throws InterruptedException {
        servoController = new ServoController(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Test preset positions
            if (gamepad1.a) {
                servoController.setToMin();
                telemetry.addData("Servo Position", "Min");
            } else if (gamepad1.b) {
                servoController.setToMax();
                telemetry.addData("Servo Position", "Max");
            } else if (gamepad1.x) {
                servoController.setToHalf();
                telemetry.addData("Servo Position", "Half");
            } else if (gamepad1.y) {
                servoController.setToQuarter();
                telemetry.addData("Servo Position", "Quarter");
            } else if (gamepad1.right_bumper) {
                servoController.setToThreeQuarter();
                telemetry.addData("Servo Position", "Three Quarter");
            }

            // Manual position control with left stick
            double manualPosition = (gamepad1.left_stick_y + 1) / 2.0; // Map -1 to 1 to 0 to 1
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                servoController.setPosition(manualPosition);
                telemetry.addData("Servo Position", "Manual: %.2f", manualPosition);
            }

            telemetry.addData("Current Position", servoController.getPosition());
            telemetry.update();

            sleep(100); // Small delay to prevent overwhelming the servo
        }
    }
}